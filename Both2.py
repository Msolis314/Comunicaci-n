# path: transceiver.py
"""
Sistema bidireccional de transferencia de archivos para nRF24L01+
Optimizado con FEC (Reed-Solomon) + compresión adaptativa y payload sólido de 32B.

- Estructura con FEC: header(6) + data(22) + RS(4) = 32.
- Estructura sin FEC: header(6) + data(26) = 32.

"""

import sys
import time
import binascii
import pathlib
import random
import threading
import zlib
import bz2
import lzma
import hashlib
from enum import Enum
from pyrf24 import RF24, RF24_PA_MAX, RF24_2MBPS, RF24_DRIVER

# Reed-Solomon
try:
    from reedsolo import RSCodec
    RS_AVAILABLE = True
except ImportError:
    print(" Advertencia: reedsolo no disponible. FEC deshabilitado.")
    print("  Instalar con: pip install reedsolo")
    RS_AVAILABLE = False

# GPIO opcional
try:
    import RPi.GPIO as GPIO
except ImportError:
    print("Advertencia: RPi.GPIO no disponible. LEDs y botón deshabilitados.")
    GPIO = None

# GPIO
BUTTON_PIN = 17
LED_GREEN = 23
LED_YELLOW = 24
LED_RED = 25

# Configuración nRF24L01+
CSN_PIN = 0
if RF24_DRIVER == "MRAA":
    CE_PIN = 15
elif RF24_DRIVER == "wiringPi":
    CE_PIN = 3
else:
    CE_PIN = 22

radio = RF24(CE_PIN, CSN_PIN)

# Direcciones
ADDR_A = b"\xE7\xE7\xE7\xE7\xE7"
ADDR_B = b"\xD7\xD7\xD7\xD7\xD7"

# Parámetros de trama
FRAME_SIZE = 32                # Límite duro de nRF24L01+
HEADER_SIZE = 6                # file_id(2) + seq_id(2) + len(1) + flags(1)

# Sin FEC: 6 + 26 = 32
DATA_BYTES = 26

# Con FEC (RS de 4 símbolos sobre header+data):
FEC_SYMBOLS = 4                # 4 bytes de paridad RS
EFFECTIVE_DATA_BYTES = 22      # 6 + 22 + 4 = 32

# TX optimizació
MAX_RETRIES = 3
RETRY_DELAY = 0.04
ACK_TIMEOUT = 1.5
MAX_ROUNDS = 8
BURST_SIZE = 15
INTER_PACKET_DELAY = 0.0008

# RX tiempos
GLOBAL_TIMEOUT = 1000
IDLE_TIMEOUT = 1.5

# Flags
FLAG_LAST = 0x01
FLAG_COMPRESSED = 0x02
FLAG_FEC = 0x08

# Compresión
COMPRESS_NONE = 0
COMPRESS_ZLIB = 1
COMPRESS_BZ2 = 2
COMPRESS_LZMA = 3

# Estados del sistema
class SystemState(Enum):
    IDLE = "idle"
    TX_ACTIVE = "transmitting"
    RX_ACTIVE = "receiving"
    COMPLETED = "completed"
    ERROR = "error"

# LEDs
class LEDController:
    def __init__(self):
        self.state = SystemState.IDLE
        self.running = True
        self.blink_thread = None
        if GPIO:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(LED_GREEN, GPIO.OUT)
            GPIO.setup(LED_YELLOW, GPIO.OUT)
            GPIO.setup(LED_RED, GPIO.OUT)
            self.blink_thread = threading.Thread(target=self._blink_loop, daemon=True)
            self.blink_thread.start()

    def _blink_loop(self):
        while self.running:
            if self.state == SystemState.IDLE:
                GPIO.output(LED_GREEN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(LED_GREEN, GPIO.LOW)
                time.sleep(0.5)
            elif self.state == SystemState.COMPLETED:
                GPIO.output(LED_RED, GPIO.HIGH)
                time.sleep(0.3)
                GPIO.output(LED_RED, GPIO.LOW)
                time.sleep(0.3)
            else:
                time.sleep(0.1)

    def set_state(self, state):
        if not GPIO:
            self.state = state
            return
        self.state = state
        GPIO.output(LED_GREEN, GPIO.LOW)
        GPIO.output(LED_YELLOW, GPIO.LOW)
        GPIO.output(LED_RED, GPIO.LOW)
        if state in (SystemState.TX_ACTIVE, SystemState.RX_ACTIVE):
            GPIO.output(LED_YELLOW, GPIO.HIGH)
        elif state == SystemState.ERROR:
            GPIO.output(LED_YELLOW, GPIO.HIGH)
            GPIO.output(LED_RED, GPIO.HIGH)

    def cleanup(self):
        self.running = False
        if self.blink_thread:
            self.blink_thread.join(timeout=1)
        if GPIO:
            GPIO.output(LED_GREEN, GPIO.LOW)
            GPIO.output(LED_YELLOW, GPIO.LOW)
            GPIO.output(LED_RED, GPIO.LOW)

# Botón
class ButtonController:
    def __init__(self, callback):
        self.callback = callback
        if GPIO:
            GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=self._button_pressed, bouncetime=300)
    def _button_pressed(self, channel):
        if self.callback:
            self.callback()

# ============= COMPRESIÓN =============
def adaptive_compress(data: bytes):
    if len(data) < 512:
        return data, COMPRESS_NONE, 1.0
    results = []
    try:
        t0 = time.time()
        c = zlib.compress(data, level=6)
        results.append((c, COMPRESS_ZLIB, len(c)/len(data), time.time()-t0, "zlib"))
    except:
        pass
    if len(data) > 5000:
        try:
            t0 = time.time()
            c = bz2.compress(data, compresslevel=5)
            results.append((c, COMPRESS_BZ2, len(c)/len(data), time.time()-t0, "bz2"))
        except:
            pass
    if len(data) > 10000:
        try:
            t0 = time.time()
            c = lzma.compress(data, preset=3)
            results.append((c, COMPRESS_LZMA, len(c)/len(data), time.time()-t0, "lzma"))
        except:
            pass
    results.append((data, COMPRESS_NONE, 1.0, 0, "none"))
    best = min(results, key=lambda x: x[2])
    if best[2] < 0.90:
        print(f"  Compresión: {best[4]} - {len(data)} → {len(best[0])} bytes (ratio: {best[2]:.2%}, tiempo: {best[3]:.3f}s)")
        return best[0], best[1], best[2]
    else:
        print(f"  Sin compresión (mejor ratio: {best[2]:.2%} con {best[4]})")
        return data, COMPRESS_NONE, 1.0

def adaptive_decompress(data: bytes, mode: int) -> bytes:
    if mode == COMPRESS_NONE:
        return data
    if mode == COMPRESS_ZLIB:
        return zlib.decompress(data)
    if mode == COMPRESS_BZ2:
        return bz2.decompress(data)
    if mode == COMPRESS_LZMA:
        return lzma.decompress(data)
    raise ValueError(f"Modo de compresión desconocido: {mode}")

# ============= FEC RS =============
if RS_AVAILABLE:
    rs_codec = RSCodec(FEC_SYMBOLS)

def apply_fec(payload_wo_rs: bytes) -> bytes:
    """Codifica RS sobre header+data. Garantiza 32B finales."""
    if not RS_AVAILABLE:
        return payload_wo_rs
    # RS agrega exactamente FEC_SYMBOLS bytes
    encoded = rs_codec.encode(payload_wo_rs)
    return encoded

def decode_fec(encoded_payload: bytes) -> tuple[bytes, int]:
    """Decodifica RS; retorna (bytes_corregidos, errores)."""
    if not RS_AVAILABLE:
        return encoded_payload, 0
    try:
        corrected, _, errors = rs_codec.decode(encoded_payload, return_stats=True)
        return bytes(corrected), errors
    except Exception:
        return encoded_payload, -1  # Señal de fallo (no truncamos)

# ============= UTILS =============
def calculate_file_hash(data: bytes) -> bytes:
    return hashlib.sha256(data).digest()[:4]

def build_frame(file_id, seq_id, data_bytes, is_last=False, compress_mode=0, use_fec=True):
    """Construye frame de 32B exactos. Con FEC: RS sobre header+data."""
    if use_fec and RS_AVAILABLE:
        max_data = EFFECTIVE_DATA_BYTES  # 22
    else:
        max_data = DATA_BYTES            # 26

    if len(data_bytes) > max_data:
        raise ValueError(f"Data excede {max_data} bytes (len={len(data_bytes)})")

    # Header
    flags = 0
    if is_last:
        flags |= FLAG_LAST
    if compress_mode > 0:
        flags |= FLAG_COMPRESSED
        flags |= (compress_mode << 4)
    if use_fec and RS_AVAILABLE:
        flags |= FLAG_FEC

    header = (
        int(file_id).to_bytes(2, 'big') +
        int(seq_id).to_bytes(2, 'big') +
        bytes([len(data_bytes)]) +
        bytes([flags])
    )

    # Data padded to max_data
    data = data_bytes + b"\x00" * (max_data - len(data_bytes))

    if use_fec and RS_AVAILABLE:
        # RS sobre (header+data) → +4B paridad
        payload_wo_rs = header + data  # 6 + 22 = 28
        encoded = apply_fec(payload_wo_rs)  # 28 + 4 = 32
        if len(encoded) != FRAME_SIZE:
            raise ValueError(f"Payload RS no es 32B (len={len(encoded)})")
        return encoded
    else:
        payload = header + data  # 6 + 26 = 32
        if len(payload) != FRAME_SIZE:
            raise ValueError(f"Payload sin FEC no es 32B (len={len(payload)})")
        return payload

def parse_frame(pkt: bytes):
    """Parsea un frame de 32B, decodifica RS si procede."""
    if len(pkt) != FRAME_SIZE:
        return None

    has_fec = False
    errors_corrected = 0
    raw = pkt

    if RS_AVAILABLE:
        decoded, errors = decode_fec(pkt)
        if errors >= 0 and len(decoded) >= HEADER_SIZE:
            has_fec = True
            errors_corrected = errors
            raw = decoded
        else:
            # Si fallo RS, intentamos interpretar (degradado)
            raw = pkt

    if len(raw) < HEADER_SIZE:
        return None

    file_id = int.from_bytes(raw[0:2], 'big')
    seq_id = int.from_bytes(raw[2:4], 'big')
    data_len = raw[4]
    flags = raw[5]

    # Si vino con FEC, los datos codificados en build_frame ocupaban 22.
    max_data = EFFECTIVE_DATA_BYTES if (flags & FLAG_FEC) else DATA_BYTES
    data_start = HEADER_SIZE
    data_end = data_start + max_data
    data = raw[data_start:data_end]

    if data_len > max_data:
        return None

    is_last = bool(flags & FLAG_LAST)
    is_compressed = bool(flags & FLAG_COMPRESSED)
    compress_mode = ((flags >> 4) & 0x0F) if is_compressed else COMPRESS_NONE

    return file_id, seq_id, data[:data_len], is_last, compress_mode, errors_corrected

def build_ack_payload(file_id, chunks, last_seq, last_seen, compress_mode=0):
    if file_id is None:
        return b"\x00\x00\xFF\xFE\x00\x00"
    COMPLETE = 1 << 0
    if last_seq is None:
        missing_seq = 0xFFFE
        flags = 0
    else:
        missing = None
        for seq in range(0, last_seq + 1):
            if seq not in chunks:
                missing = seq
                break
        if missing is None:
            missing_seq = 0xFFFF
            flags = COMPLETE if last_seen else 0
        else:
            missing_seq = missing
            flags = 0
    return (
        int(file_id).to_bytes(2, 'big') +
        int(missing_seq).to_bytes(2, 'big') +
        bytes([flags]) +
        bytes([compress_mode])
    )

def parse_ack(ack_data):
    if len(ack_data) < 5:
        return None, None, False, 0
    file_id = int.from_bytes(ack_data[0:2], 'big')
    missing_seq = int.from_bytes(ack_data[2:4], 'big')
    flags = ack_data[4]
    compress_mode = ack_data[5] if len(ack_data) > 5 else 0
    is_complete = bool(flags & 0x01)
    if missing_seq in (0xFFFF, 0xFFFE):
        missing_seq = None
    return file_id, missing_seq, is_complete, compress_mode

def split_file(file_path, use_fec=True):
    data = file_path.read_bytes()
    original_size = len(data)
    file_hash = calculate_file_hash(data)
    compressed, compress_mode, ratio = adaptive_compress(data)
    final_size = len(compressed)
    chunk_size = EFFECTIVE_DATA_BYTES if (use_fec and RS_AVAILABLE) else DATA_BYTES
    chunks = [compressed[i:i+chunk_size] for i in range(0, len(compressed), chunk_size)]
    return chunks, compress_mode, original_size, final_size, file_hash

# ============= TRANSMISOR =============
def transmit_file(radio: RF24, file_path: pathlib.Path, led_controller):
    print("\n[ MODO TRANSMISOR ]")
    led_controller.set_state(SystemState.TX_ACTIVE)
    try:
        # Pipes
        radio.open_rx_pipe(1, ADDR_B)
        radio.stop_listening()  # por si acaso
        radio.open_tx_pipe(ADDR_A)
        radio.set_auto_retries(5, 5)

        file_id = random.randint(0, 65535)
        print(f"\n{'='*50}")
        print("MODO TRANSMISOR (OPT)")
        print(f"{'='*50}")
        print(f"Archivo: {file_path.name}")
        print(f"File ID: {file_id}")
        print(f"Tamaño original: {file_path.stat().st_size} bytes")

        start_prep = time.time()
        chunks, compress_mode, original_size, final_size, file_hash = split_file(file_path, use_fec=RS_AVAILABLE)
        prep_time = time.time() - start_prep

        compress_names = {0: "none", 1: "zlib", 2: "bz2", 3: "lzma"}
        total_packets = len(chunks)
        chunk_size = EFFECTIVE_DATA_BYTES if RS_AVAILABLE else DATA_BYTES

        print(f"Tamaño procesado: {final_size} bytes")
        print(f"Compresión: {compress_names.get(compress_mode, 'unknown')}")
        print(f"Total paquetes: {total_packets}")
        print(f"Bytes por paquete (datos): {chunk_size}")
        print(f"FEC: {'Habilitado' if RS_AVAILABLE else 'Deshabilitado'}")
        print(f"Hash (4B): {file_hash.hex()}")
        print(f"Tiempo preparación: {prep_time:.3f}s")

        pending = set(range(total_packets))
        sent_count = 0
        success_count = 0
        start_time = time.time()
        burst_stats = {'sent': 0, 'ack': 0, 'fail': 0}

        for round_num in range(MAX_ROUNDS):
            if not pending:
                print("✓ Todos los paquetes confirmados!")
                break

            print(f"\n--- Ronda {round_num + 1} ---")
            print(f"Pendientes: {len(pending)}")
            pending_list = sorted(pending)

            for burst_start in range(0, len(pending_list), BURST_SIZE):
                burst_end = min(burst_start + BURST_SIZE, len(pending_list))
                burst = pending_list[burst_start:burst_end]

                for seq_id in burst:
                    is_last = (seq_id == total_packets - 1)
                    frame = build_frame(file_id, seq_id, chunks[seq_id], is_last, compress_mode, RS_AVAILABLE)

                    success = False
                    ack_payload = None

                    for attempt in range(MAX_RETRIES):
                        if radio.write(frame):
                            success = True
                            burst_stats['sent'] += 1
                            # Si hay ACK payload disponible, leerlo
                            if radio.available():
                                try:
                                    size = radio.get_dynamic_payload_size()
                                    if 0 < size <= 32:
                                        ack_payload = radio.read(size)
                                        burst_stats['ack'] += 1
                                except:
                                    pass
                            break
                        time.sleep(RETRY_DELAY)

                    if success:
                        sent_count += 1
                        success_count += 1
                        pending.discard(seq_id)  # ← FIX fundamental

                        if sent_count % 25 == 0 or is_last:
                            progress = (sent_count / total_packets) * 100
                            elapsed = time.time() - start_time
                            # Throughput “efectivo” de datos (no cuenta encabezado/RS)
                            throughput_kibs = (sent_count * chunk_size) / max(elapsed, 1e-9) / 1024
                            print(f"  📊 {progress:.1f}% | {sent_count}/{total_packets} | {throughput_kibs:.1f} KiB/s")

                        if ack_payload:
                            _, missing_seq, is_complete, _ = parse_ack(ack_payload)
                            if is_complete:
                                pending.clear()
                                break
                            if missing_seq is not None and missing_seq in pending:
                                # Opcional: priorizar el faltante reportado
                                pass
                    else:
                        burst_stats['fail'] += 1

                    time.sleep(INTER_PACKET_DELAY)

                if not pending:
                    break

            if pending:
                # “Ping” con último paquete para forzar estado
                time.sleep(0.3)
                last_seq = total_packets - 1
                frame = build_frame(file_id, last_seq, chunks[last_seq], True, compress_mode, RS_AVAILABLE)
                if radio.write(frame) and radio.available():
                    try:
                        size = radio.get_dynamic_payload_size()
                        if 0 < size <= 32:
                            ack_payload = radio.read(size)
                            _, missing_seq, is_complete, _ = parse_ack(ack_payload)
                            if is_complete:
                                print("✓ Receptor confirma recepción completa!")
                                pending.clear()
                                break
                            elif missing_seq is not None:
                                # Mantén solo lo que falta desde el primer missing
                                pending = set([s for s in pending if s >= missing_seq])
                    except:
                        pass

        total_time = time.time() - start_time

        if not pending:
            throughput_orig = (original_size / max(total_time, 1e-9)) / 1024
            efficiency = (success_count / sent_count * 100) if sent_count > 0 else 0
            compression_ratio = final_size / original_size if original_size > 0 else 1.0
            print(f"\n{'='*50}")
            print("✓ ¡TRANSMISIÓN EXITOSA!")
            print(f"{'='*50}")
            print(f"Tiempo total: {total_time:.2f}s")
            print(f"Throughput (original): {throughput_orig:.2f} KiB/s")
            print(f"Paquetes enviados: {sent_count} (únicos: {success_count})")
            print(f"Eficiencia: {efficiency:.1f}%")
            print(f"Ratio compresión: {compression_ratio:.2%}")
            print(f"Bytes ahorrados: {original_size - final_size}")
            print("Estadísticas burst:")
            print(f"  - Enviados: {burst_stats['sent']}")
            print(f"  - ACKs recibidos: {burst_stats['ack']}")
            print(f"  - Fallos: {burst_stats['fail']}")
            if RS_AVAILABLE:
                print(f"FEC: Activo (RS {FEC_SYMBOLS} bytes paridad)")
            print(f"{'='*50}\n")
            led_controller.set_state(SystemState.COMPLETED)
            return True
        else:
            print(f"\n{'='*50}")
            print("✗ TRANSMISIÓN INCOMPLETA")
            print(f"Faltantes: {len(pending)}")
            print(f"Tiempo: {total_time:.2f}s")
            print(f"{'='*50}\n")
            led_controller.set_state(SystemState.ERROR)
            return False

    except Exception as e:
        print(f"\n✗ Error en transmisión: {e}")
        import traceback; traceback.print_exc()
        led_controller.set_state(SystemState.ERROR)
        return False

# ============= RECEPTOR =============
def receive_file(radio: RF24, dest_dir: pathlib.Path, led_controller):
    print("\n[ MODO RECEPTOR ]")
    led_controller.set_state(SystemState.RX_ACTIVE)
    try:
        radio.open_rx_pipe(1, ADDR_A)
        radio.open_tx_pipe(ADDR_B)
        radio.start_listening()

        print(f"\n{'='*50}")
        print("MODO RECEPTOR (OPT)")
        print(f"{'='*50}")
        print(f"Directorio: {dest_dir.absolute()}")
        print(f"FEC: {'Habilitado' if RS_AVAILABLE else 'Deshabilitado'}")
        print("Esperando datos...\n")

        file_id_seen = None
        chunks = {}
        last_seq = None
        last_seen = False
        compress_mode = COMPRESS_NONE
        start_time = time.monotonic()
        last_packet_time = None
        packets_received = 0
        total_errors_corrected = 0

        first_ack = build_ack_payload(file_id_seen, chunks, last_seq, last_seen)
        radio.write_ack_payload(1, first_ack)

        while True:
            now = time.monotonic()
            if (now - start_time) > GLOBAL_TIMEOUT:
                print("⏱ Timeout global")
                break
            if last_seen and last_packet_time is not None:
                if (now - last_packet_time) > IDLE_TIMEOUT:
                    print("⏱ Timeout idle")
                    break

            has_payload, pipe = radio.available_pipe()
            if not has_payload:
                time.sleep(0.001)
                continue

            try:
                payload_size = radio.get_dynamic_payload_size()
            except:
                payload_size = 0

            if payload_size == 0 or payload_size > FRAME_SIZE:
                try:
                    radio.read(payload_size if payload_size > 0 else 32)
                except:
                    pass
                ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen, compress_mode)
                radio.write_ack_payload(1, ack_payload)
                continue

            raw = radio.read(payload_size)
            if len(raw) < FRAME_SIZE:
                raw += b"\x00" * (FRAME_SIZE - len(raw))

            parsed = parse_frame(raw)
            if parsed is None:
                ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen, compress_mode)
                radio.write_ack_payload(1, ack_payload)
                continue

            fid, seq_id, data_bytes, is_last, pkt_compress, errors = parsed
            last_packet_time = now
            packets_received += 1
            if errors > 0:
                total_errors_corrected += errors

            if file_id_seen is None:
                file_id_seen = fid
                compress_mode = pkt_compress
                compress_names = {0: "none", 1: "zlib", 2: "bz2", 3: "lzma"}
                print(f"→ ID {file_id_seen} | Compresión: {compress_names.get(compress_mode, 'unknown')}\n")

            if fid != file_id_seen:
                ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen, compress_mode)
                radio.write_ack_payload(1, ack_payload)
                continue

            if seq_id not in chunks:
                chunks[seq_id] = data_bytes
                if packets_received % 25 == 0 or is_last:
                    progress = len(chunks)
                    elapsed = time.monotonic() - start_time
                    # Throughput de datos útiles
                    per_pkt = len(data_bytes)
                    throughput = (progress * per_pkt) / max(elapsed, 1e-9) / 1024
                    print(f"  📊 {progress} paquetes | {throughput:.1f} KiB/s | Errores FEC: {total_errors_corrected}")

            if is_last:
                last_seq = seq_id
                last_seen = True
                print(f"\n→ Último paquete: {last_seq}")
                print(f"  Total recibidos: {len(chunks)} de {last_seq + 1}")

            ack_payload = build_ack_payload(file_id_seen, chunks, last_seq, last_seen, compress_mode)
            radio.write_ack_payload(1, ack_payload)

        radio.stop_listening()
        total_time = time.monotonic() - start_time

        if not chunks:
            print("\n✗ No se recibieron datos")
            led_controller.set_state(SystemState.ERROR)
            return False

        print(f"\n{'='*50}")
        print("RECONSTRUYENDO ARCHIVO")
        print(f"{'='*50}")

        max_seq = max(chunks.keys())
        reconstructed = bytearray()
        missing = []
        for s in range(0, max_seq + 1):
            if s in chunks:
                reconstructed += chunks[s]
            else:
                missing.append(s)

        if missing:
            print(f"⚠ Paquetes faltantes: {len(missing)}")
            head = ','.join(map(str, missing[:20]))
            print(f"  Lista: {head}{'...' if len(missing) > 20 else ''}")

        original_size = len(reconstructed)
        if compress_mode != COMPRESS_NONE:
            print("Descomprimiendo datos...")
            try:
                decompressed = adaptive_decompress(bytes(reconstructed), compress_mode)
                reconstructed = bytearray(decompressed)
                print(f"  {original_size} → {len(reconstructed)} bytes")
            except Exception as e:
                print(f"✗ Error al descomprimir: {e}")
                led_controller.set_state(SystemState.ERROR)
                return False

        timestamp = int(time.time())
        filename = f"file_{file_id_seen}_{timestamp}.bin" if file_id_seen else f"file_{timestamp}.bin"
        dest_path = dest_dir / filename
        dest_path.write_bytes(reconstructed)

        throughput = (len(reconstructed) / max(total_time, 1e-9)) / 1024
        print(f" Archivo: {dest_path.name}")
        print(f" Tamaño final: {len(reconstructed)} bytes")
        print(f" Paquetes: {len(chunks)}/{max_seq + 1}")
        print(f" Tiempo: {total_time:.2f}s")
        print(f" Throughput: {throughput:.2f} KiB/s")
        if total_errors_corrected > 0:
            print(f" Errores corregidos por FEC (RS): {total_errors_corrected}")
        print(f" Faltantes: {len(missing)}")

        if missing:
            print(f"{'='*50}\n")
            led_controller.set_state(SystemState.ERROR)
            return False
        else:
            print("✓ ¡Recepción completa sin pérdidas!")
            print(f"{'='*50}\n")
            led_controller.set_state(SystemState.COMPLETED)
            return True

    except Exception as e:
        print(f"\n✗ Error en recepción: {e}")
        import traceback; traceback.print_exc()
        led_controller.set_state(SystemState.ERROR)
        radio.stop_listening()
        return False


def main():
    if len(sys.argv) != 3:
        print("Uso:")
        print("  python3 transceiver.py <archivo_a_enviar> <directorio_recepcion>")
        print("\nPresione el botón para alternar entre TX y RX")
        sys.exit(1)

    file_path = pathlib.Path(sys.argv[1])
    dest_dir = pathlib.Path(sys.argv[2])
    if not file_path.is_file():
        print(f"Error: {file_path} no es un archivo válido")
        sys.exit(1)
    if not dest_dir.is_dir():
        print(f"Error: {dest_dir} no es un directorio válido")
        sys.exit(1)

    # Radio
    if not radio.begin():
        print("✗ Error al inicializar nRF24L01+")
        sys.exit(1)
    radio.set_pa_level(RF24_PA_MAX)
    radio.dynamic_payloads = True
    radio.ack_payloads = True
    radio.channel = 90
    radio.data_rate = RF24_2MBPS
    radio.set_auto_retries(5, 15)

    # Controladores
    led_controller = LEDController()
    mode = {'current': 'idle'}

    def toggle_mode():
        if mode['current'] == 'idle':
            mode['current'] = 'tx'
            print("\n BOTÓN → Iniciando TRANSMISIÓN (TX)")
        elif mode['current'] == 'tx':
            mode['current'] = 'rx'
            print("\n BOTÓN → Cambiando a RECEPCIÓN (RX)")
        else:
            mode['current'] = 'tx'
            print("\nBOTÓN → Cambiando a TRANSMISIÓN (TX)")

    button_controller = ButtonController(toggle_mode)

    print("="*70)
    print("SISTEMA DE TRANSFERENCIA BIDIRECCIONAL nRF24L01+")
    print("="*70)
    print("\nOPTIMIZACIONES ACTIVAS:")
    print(f"   Payload : {FRAME_SIZE} bytes (límite nRF24)")
    if RS_AVAILABLE:
        print(f" FEC RS: {FEC_SYMBOLS} símbolos (6+22+4=32)")
    else:
        print("  FEC deshabilitado (instalar: pip install reedsolo)")
    print("  Compresión adaptativa: zlib, bz2, lzma")
    print(" Data rate: 2 MBPS")
    print(f"  Modo ráfaga: {BURST_SIZE} paquetes/burst")
    print(f" Delay ultra-bajo: {INTER_PACKET_DELAY*1000:.1f}ms entre paquetes")
    print("\n ESTADOS LED:")
    print("  🟢 Verde parpadeando: Idle")
    print("  🟡 Amarillo fijo: Transferencia")
    print("  🔴 Rojo parpadeando: Completado")
    print("  🟡🔴 Amarillo+Rojo: Error")
    print("\nCONTROL:")
    print("  Presione el botón para alternar TX/RX")
    print("  Presione Ctrl+C para salir\n")
    print("="*70)

    try:
        while True:
            if mode['current'] == 'tx':
                print("\n>> Entrando a modo TRANSMISOR (TX) <<")
                transmit_file(radio, file_path, led_controller)
                time.sleep(3)
                mode['current'] = 'idle'
                led_controller.set_state(SystemState.IDLE)
                print("\n💤 Idle - Presione botón para nueva transmisión\n")
            elif mode['current'] == 'rx':
                print("\n>> Entrando a modo RECEPTOR (RX) <<")
                receive_file(radio, dest_dir, led_controller)
                time.sleep(3)
                mode['current'] = 'idle'
                led_controller.set_state(SystemState.IDLE)
                print("\nIdle - Presione botón para cambiar a TX\n")
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\nSistema detenido por el usuario")
    finally:
        led_controller.cleanup()
        if GPIO:
            GPIO.cleanup()
        print("✓ Limpieza completada\n")

if __name__ == "__main__":
    main()
