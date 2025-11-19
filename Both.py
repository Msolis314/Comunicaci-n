'''
Receptor de archivos para el modulo nRF24L01+

Formato de la trama:
[file_id (2 bytes)][seq_id (2 bytes)][len (1 byte)][flags (1 byte)][data (24 bytes)][crc (2 bytes)]

Donde:
- file_id: Identificador unico del archivo
- seq_id: Numero de secuencia del paquete
- len: bytes utiles en data
- flags: bit 0 = last packet
- data: datos del paquete
- crc: checksum del paquete
'''

import sys
import time
import binascii
import pathlib
from pyrf24 import RF24, RF24_PA_MAX, RF24_1MBPS, RF24_DRIVER


# Configuracion
CSN_PIN = 0

if RF24_DRIVER == "MRAA":
    CE_PIN = 15  # Pin GPIO22 de la Raspberry Pi
elif RF24_DRIVER == "wiringPi":
    CE_PIN = 3  # Pin GPIO22 de la Raspberry Pi
else:
    CE_PIN = 25  # Pin GPIO22 de la Raspberry Pi

radio = RF24(CE_PIN, CSN_PIN)

# Direcciones de pines (INVERTIDAS respecto al transmisor)
TX_ADDR = b"\xD7\xD7\xD7\xD7\xD7"  # Para enviar ACKs
RX_ADDR = b"\xE7\xE7\xE7\xE7\xE7"  # Para recibir datos

# Tams de tramas
FRAME_SIZE = 32
DATA_BYTES = 24

# Timeouts
GLOBAL_TIMEOUT = 1000  # Tiempo maximo de recepcion del archivo
IDLE_TIMEOUT = 2.0  # Tiempo sin paquetes despues de ver last para reconstruir


# Utilidades

def crc16(data: bytes, init: int = 0xFFFF) -> int:
    '''CRC-CCITT usando binascii.crc_hqx'''
    return binascii.crc_hqx(data, init)


def parse_frame(pkt: bytes):
    '''Parsear una trama de 32 bytes y verificar CRC
    
    Returns: (file_id, seq_id, data_bytes, last_flag) o None si falla el CRC
    '''
    if len(pkt) != FRAME_SIZE:
        return None
    
    file_id = int.from_bytes(pkt[0:2], 'big')
    seq_id = int.from_bytes(pkt[2:4], 'big')
    data_len = pkt[4]
    flags = pkt[5]
    data = pkt[6:6 + DATA_BYTES]
    crc_rx = int.from_bytes(pkt[30:32], 'big')
    
    # Calcular CRC de los primeros 30 bytes
    crc_calc = crc16(pkt[0:30])
    
    if crc_rx != crc_calc:
        # CRC incorrecto, se descarta la trama
        return None
    
    last = bool(flags & 0x01)
    
    return file_id, seq_id, data[:data_len], last


def build_ack_payload(file_id, chunks, last_seq, last_seen):
    '''Construir la carga util del ACK compacta
    [file_id: 2][missing_seq: 2][flags: 1] -> 5 bytes
    
    - missing_seq:
        - 0xFFFF si no faltan paquetes
        - 0xFFFE si aun no sabe el ultimo paquete
        - seq_id del primer paquete faltante
    - flags:
        - bit 0: transmision completa
    '''
    
    if file_id is None:
        # Nada recibido aun
        return b"\x00\x00\xFF\xFE\x00"
    
    COMPLETE = 1 << 0
    
    if last_seq is None:
        # Todavia no se sabe el ultimo paquete
        missing_seq = 0xFFFE
        flags = 0
    else:
        # Buscar el primer faltante
        missing = None
        for seq in range(0, last_seq + 1):
            if seq not in chunks:
                missing = seq
                break
        
        if missing is None:
            # No hay faltantes
            missing_seq = 0xFFFF
            flags = COMPLETE if last_seen else 0
        else:
            # Hay faltantes
            missing_seq = missing
            flags = 0
    
    return (
        int(file_id).to_bytes(2, 'big') +
        int(missing_seq).to_bytes(2, 'big') +
        bytes([flags])
    )


def main():
    if len(sys.argv) != 2:
        print("Uso: python3 rx.py <directorio_destino>")
        sys.exit(1)
    
    dest_dir = pathlib.Path(sys.argv[1])
    if not dest_dir.is_dir():
        print(f"Error: {dest_dir} no es un directorio valido")
        sys.exit(1)
    
    # Inicializar radio
    if not radio.begin():
        print("Error al inicializar el modulo nRF24L01+")
        sys.exit(1)
    
    # Configuracion del radio
    radio.set_pa_level(RF24_PA_MAX)  # Maxima potencia
    radio.dynamic_payloads = True  # Payloads dinamicos
    radio.ack_payloads = True  # Habilitar ACK con payloads
    radio.channel = 90          
    radio.data_rate = RF24_1MBPS
    
    # Configurar direcciones
    radio.open_rx_pipe(1, RX_ADDR)  # Pipe 1 para recepcion
    radio.open_tx_pipe(TX_ADDR)  # Para enviar ACKs
    
    # Modo recepcion
    radio.listen = True
    
    # Variables para reconstruir el archivo
    file_id_seen = None
    chunks = {}  # seq_id -> data
    last_seq = None
    last_seen = False
    start_time = time.monotonic()
    last_packet_time = None
    
    # Pre-cargar un primer ACK payload vacio
    first_ack = build_ack_payload(file_id_seen, chunks, last_seq, last_seen)
    radio.write_ack_payload(1, first_ack)
    
    print("Receptor iniciado. Esperando datos...")
    print(f"Directorio destino: {dest_dir.absolute()}")
    
    try:
        while True:
            now = time.monotonic()
            
            # Timeout global
            if (now - start_time) > GLOBAL_TIMEOUT:
                print("Timeout global alcanzado, terminando recepcion")
                break
            
            # Condicion para reconstruir el archivo
            if last_seen and last_packet_time is not None:
                if (now - last_packet_time) > IDLE_TIMEOUT:
                    print("Timeout idle alcanzado, reconstruyendo archivo")
                    break
            
            # Verificar si hay datos disponibles
            has_payload, pipe = radio.available_pipe()
            
            if not has_payload:
                # No hay paquete nuevo, ceder CPU un momento
                time.sleep(0.001)
                continue
            
            # Leer paquete
            payload_size = radio.get_dynamic_payload_size()
            
            if payload_size == 0 or payload_size > FRAME_SIZE:
                # Tamano invalido, descartar
                radio.read(payload_size if payload_size > 0 else 32)
                # Preparar un ACK generico
                ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen)
                radio.write_ack_payload(1, ack_payload)
                continue
            
            raw = radio.read(payload_size)
            
            # Si viene mas corto, rellenar con ceros
            if len(raw) < FRAME_SIZE:
                raw += b"\x00" * (FRAME_SIZE - len(raw))
            
            # Parsear trama
            parsed = parse_frame(raw)
            
            if parsed is None:
                print("Paquete recibido con CRC invalido, descartando")
                # Preparar un ACK generico
                ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen)
                radio.write_ack_payload(1, ack_payload)
                continue
            
            fid, seq_id, data_bytes, is_last = parsed
            last_packet_time = now
            
            if file_id_seen is None:
                file_id_seen = fid
                print(f"Iniciando recepcion del archivo ID {file_id_seen}")
            
            if fid != file_id_seen:
                # Se ignora paquete de otro archivo
                print(f"Paquete recibido de archivo ID {fid} ignorado (esperando ID {file_id_seen})")
                ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen)
                radio.write_ack_payload(1, ack_payload)
                continue
            
            # Evitar duplicados
            if seq_id in chunks:
                print(f"Paquete duplicado recibido seq_id {seq_id}, ignorando")
            else:
                chunks[seq_id] = data_bytes
                print(f"Paquete recibido seq_id {seq_id}, bytes utiles {len(data_bytes)}, last: {is_last}")
            
            if is_last:
                last_seq = seq_id
                last_seen = True
                print(f"Marcado Last seq_id {last_seq}")
            
            # Preparar ACK con estado actualizado
            ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen)
            radio.write_ack_payload(1, ack_payload)
    
    except KeyboardInterrupt:
        print("\nRecepcion interrumpida por el usuario")
    
    finally:
        # Volver a modo inactivo
        radio.listen = False
    
    # Reconstruir archivo
    if not chunks:
        print("No se recibieron datos, no se reconstruye ningun archivo")
        return
    
    print("\n--- Reconstruyendo archivo ---")
    
    # Reconstruir datos en orden
    max_seq = max(chunks.keys())
    reconstructed = bytearray()
    missing_count = 0
    missing_list = []
    
    for s in range(0, max_seq + 1):
        if s in chunks:
            reconstructed += chunks[s]
        else:
            missing_count += 1
            missing_list.append(s)
            print(f"Paquete seq_id {s} faltante en la reconstruccion")
    
    # Guardar archivo con timestamp
    timestamp = int(time.time())
    if file_id_seen is not None:
        filename = f"file_{file_id_seen}_{timestamp}.bin"
    else:
        filename = f"file_{timestamp}.bin"
    
    dest_path = dest_dir / filename
    dest_path.write_bytes(reconstructed)
    
    print(f"\n--- Resultado ---")
    print(f"Archivo guardado en: {dest_path}")
    print(f"Total bytes: {len(reconstructed)}")
    print(f"Paquetes unicos recibidos: {len(chunks)}/{max_seq + 1}")
    print(f"Paquetes faltantes: {missing_count}")
    
    if missing_count > 0:
        print(f"Lista de faltantes: {missing_list}")
    else:
        print("¡Recepcion completa sin perdidas!")
    
    print("Fin de la recepcion.")


if __name__ == "__main__":
    main()
