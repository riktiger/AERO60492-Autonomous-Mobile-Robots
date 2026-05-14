#!/usr/bin/env python3

# AERO60492 & AERO40492 Autonomous Mobile Robots 2025/26 2nd Semester
# CW1 (Coursework 1) - Serial Message Decoder
# Submitted by Aritra Bag (Student ID - 14320390)



# ------------------------------------------------------------
# SOLUTIONS TO THE GIVEN QUESTIONS FOR FILE binaryFileC_10.bin
# ------------------------------------------------------------
# Q1 Answer: The number of complete data frames in file binaryFileC_10.bin is 499
# Q2 Answer: The number of potentially corrupt data frames (messages) in file binaryFileC_10.bin is 16 
#            Corruption in this instance is accounted for all complete frames with checksum errors. 
#            1 additional frame is incomplete. Incomplete in this instance consists of missing bytes, i.e. a dataframe with less than 26 bytes.
#            Total number of frames with decoding errors is thus 17.
# Q3 Answer: The calendar date of these messages is 1998-10-08 (format YYYY-MM-DD, October 8, 1998)



# ------------------------------------------------------------
# PROBLEM STATEMENT AND SCRIPT FUNCTION DESCRIPTION
# ------------------------------------------------------------

# This script decrypts the binary encodings of serial data to human readable form, stores them as .csv file and logs the corrupted
# frames along with their detected errors. It counts the number of complete frames, the number of corrupt and incomplete frames and the date and
# time of when the data was collected.The final expeceted format of each dataframe is 26 bytes long with the following structure -
#
# HEADER SECTION
#        Byte 1 : Value = ~ : The first of two sequential ~ characters indicate the start of a new data frame
#        Byte 2 : Value = ~ : The second of two sequential ~ characters indicate the start of a new data frame
#        Byte 3 : Value = (0-255) : SYS ID, An integer value indicating which device sent this data 0-255
#        Byte 4 : Value = (0-255) : DEST ID, An integer value indicating the device this data was destined for 0-255
#        Byte 5 : Value = (0-255) : COMP ID, An integer value indicating which sub-component sent this data 0-255
#        Byte 6 : Value = (0-255) : SEQ, A value which should increment by one each data frame. Used to allow the detection of missing frames.
#        Byte 7 : Value = (0-255) : TYPE, An ID value indicating what type of data is in the payload.
# PAYLOAD SECTION
#        Byte 8 : Value = P       : PTX, A value indicating the end of the message header and start of the payload. 
#        Byte 9 : Value =  (0-255): RPM MSB, Most and least significant bytes for RPM measurement 
#        Byte 10 : Value = (0-255) : RPM LSB, as unsigned 16-bit unsigned integer. Unit: rev per minute
#        Byte 11 : Value = (0-255) : VLT MSB, Most and least significant bytes for voltage measurement
#        Byte 12 : Value = (0-255) : VLT LSB,as unsigned 16-bit unsigned integer. Unit: millivolt
#        Byte 13 : Value = (0-255) : CRT LSB, Least and most significant bytes for current
#        Byte 14 : Value = (0-255) : CRT MSB, measurement as signed 16-bit integer. Unit: milliamp
#        Byte 15 : Value = (0-255) : MOS TMP, Temperature of mosfets. Uses lookup table to interpret value.
#        Byte 16 : Value = (0-255) : CAP TMP, Temperature of capacitors. Uses lookup table to interpret value.
# TIMING SECTION
#        Byte 17 : Value = T       : TTX, A value indicating the end of the message payload and indicating the start of the timing information. 
#        Byte 18 : Value = (0-255) : TIME B0, A timestamp expressed as a Unix time code as a 64 bit 
#        Byte 19 : Value = (0-255) : TIME B1, unsigned integer. B0 is most significant bit, B7 is least
#        Byte 20 : Value = (0-255) : TIME B2, significant bit. Unit: microseconds
#        Byte 21 : Value = (0-255) : TIME B3
#        Byte 22 : Value = (0-255) : TIME B4 
#        Byte 23 : Value = (0-255) : TIME B5 
#        Byte 24 : Value = (0-255) : TIME B6 
#        Byte 25 : Value = (0-255) : TIME B7
# CHECKSUM SECTION
#        Byte 26 : Value = (0-255) : CHKSUM, Checksum value used to indicate if a data frame has become corrupted.



# ------------------------------------------------------------
# DECODING APPROACH SUMMARY
# ------------------------------------------------------------
#
# This script employs a three-state machine (SEEK, STX1, READ) to synchronize with 
# the binary stream, ensuring frame alignment by identifying the dual 0x7E header.
# Data extraction accounts for mixed endianness, specifically utilizing big-endian
# assembly for RPM and Voltage, and little-endian signed integer conversion for  
# Current. Data integrity is validated through a multi-layered
# approach: checking PTX/TTX constants, performing temperature lookup verification,
# and calculating a 256-bit modular checksum: $$Checksum = (255 - (\sum_{i=0}^{24} Byte_i \pmod{256})) \pmod{256}$$.
# The final output is bifurcated into a primary telemetry CSV and a detailed corruption log for forensic diagnostic analysis. 


# ------------------------------------------------------------
# IMPORTS
# ------------------------------------------------------------
# Datetime is required for converting the 8-byte timestamp field into a human-readable ISO-8601 format. 
# The timestamp is interpreted as microseconds since the Unix epoch, hence division by 1,000,000 is required.

from datetime import datetime, timezone


# ------------------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------
# Input binary file and output CSV files. These filenames can be modified as needed.

input_path = "binaryFileC_10.bin"
output_csv = "14320390.csv"
detailed_csv = "detailed_corrupt_frame_logs.csv"


# A fixed frame length of 26 bytes as defined by the protocol.

FRAME_LEN = 26


# ------------------------------------------------------------
# TEMPERATURE LOOKUP TABLE
# ------------------------------------------------------------
# The MOS and CAP temperature bytes must map to a valid entry in this lookup table as per the protocol.
# Any value not present here constitutes a protocol violation and must be treated as corruption, indicated by an incorrect checksum.

temperature_table = {
    0xA0: 30.0, 0xA1: 30.1, 0xA2: 30.2, 0xA3: 30.3, 0xA4: 30.4,
    0xA5: 30.5, 0xA6: 30.6, 0xA7: 30.7, 0xA8: 30.8, 0xA9: 30.9,
    0xAA: 31.0, 0xAB: 31.1, 0xAC: 31.2, 0xAD: 31.3, 0xAE: 31.4,
    0xAF: 31.5, 0xB0: 31.6, 0xB1: 31.7, 0xB2: 31.8, 0xB3: 31.9,
    0xB4: 32.0, 0xB5: 32.1, 0xB6: 32.2, 0xB7: 32.3, 0xB8: 32.4,
    0xB9: 32.5, 0xBA: 32.6, 0xBB: 32.7, 0xBC: 32.8, 0xBD: 32.9,
    0xBE: 33.0, 0xBF: 33.1, 0xC0: 33.2, 0xC1: 33.3, 0xC2: 33.4,
    0xC3: 33.5, 0xC4: 33.6, 0xC5: 33.7, 0xC6: 33.8, 0xC7: 33.9,
    0xC8: 34.0, 0xC9: 34.1, 0xCA: 34.2, 0xCB: 34.3, 0xCC: 34.4,
    0xCD: 34.5, 0xCE: 34.6, 0xCF: 34.7, 0xD0: 34.8, 0xD1: 34.9,
    0xD2: 35.0, 0xD3: 35.1, 0xD4: 35.2, 0xD5: 35.3, 0xD6: 35.4,
    0xD7: 35.5, 0xD8: 35.6, 0xD9: 35.7, 0xDA: 35.8, 0xDB: 35.9,
    0xDC: 36.0, 0xDD: 36.1, 0xDE: 36.2, 0xDF: 36.3
}


# ------------------------------------------------------------
# HELPER FUNCTIONS
# ------------------------------------------------------------
def printable_char(b: int) -> str:

    """
    Converts a byte into a printable character if it is ASCII.
    Otherwise returns a hex literal. 
    This is used for PTX/TTX fields.
    """
    return chr(b) if 32 <= b <= 126 else f"0x{b:02X}"

def frame_to_hex(frame) -> str:

    """
    Converts a list of bytes into a continuous uppercase hex string.
    This is used in the corruption summary and detailed_corrupt_frame_logs.csv.
    """
    
    return ''.join(f"{b:02X}" for b in frame)

def ts_to_iso(ts_raw: int):

    """
    Converts an 8-byte timestamp (microseconds since epoch) into ISO-8601.
    If conversion fails, returns None. Dual representation (raw + ISO) is used for the 
    corruption summary for easier interpretataion and tracking.
    """

    try:
        dt = datetime.fromtimestamp(ts_raw / 1_000_000.0, tz=timezone.utc)
        return dt.strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
    except Exception:
        return None


# ------------------------------------------------------------
# FILE OPENING
# ------------------------------------------------------------
# The binary file is opened in raw mode. The CSV files are opened for writing.
# Columns are added to the detailed corrupt frame logs csv for easier interpretataion.

fin = open(input_path, "rb")
fout = open(output_csv, "w")
fd = open(detailed_csv, "w")
fd.write("sequence_number,frame_index,complete_frame,frame_hex,ptx_hex,ptx_char,ttx_hex,ttx_char,checksum_calc,checksum_recv,mos_raw,cap_raw,mos_tmp,cap_tmp,ts_raw,ts_iso,issues\n")


# ------------------------------------------------------------
# STATE MACHINE INITIALIZATION
# ------------------------------------------------------------
# The protocol defines a strict 3-state machine to ensures correct frame alignment and prevents decoding of misaligned data.:
#   SEEK  → looking for first 0x7E
#   STX1 → first 0x7E found, expecting second 0x7E
#   READ → reading the remaining 24 bytes of the frame

state = "SEEK"
frame_buf = []
frame_index = 0
complete_frames = 0
corrupt_frames = 0
incomplete_frames = 0
timestamps = []

# This list stores all corrupted frames for the final summary.
corrupted_records = []


# ------------------------------------------------------------
# MAIN BYTE-BY-BYTE LOOP
# ------------------------------------------------------------
# EVERY byte is read and then printed as decimal and hex (as raw bytes) to mimic the serial decoding operation.

byte = fin.read(1)
while byte:
    b = byte[0]

    # Decimal and Hexadecimal byte prints:
    print(f"Byte value is (decimal): {b}")
    print(f"Byte value is (hexidecimal): {byte}")

    # --------------------------------------------------------
    # STATE MACHINE LOGIC
    # --------------------------------------------------------
    # SEEK STATE:
    # THe decoder is not currently inside a frame.It searches the first 0x7E,which may indicate the start of a frame. 
    # If found, it transitions to STX1.
    if state == "SEEK":
        if b == 0x7E:
            frame_buf = [b]
            state = "STX1"

    # STX1 STATE:
    # After finding the first 0x7E,the protocol requires a second 0x7E.
    # If the next byte is 0x7E, the frame start is confirmed and the decoder transitions to READ.
    # Else, it resets to SEEK to maintain alignment integrity.        
    elif state == "STX1":
        if b == 0x7E:
            frame_buf.append(b)
            state = "READ"
        else:
            state = "SEEK"
            frame_buf = []
    
    # READ STATE:
    # Once the frame start (0x7E 0x7E) has been confirmed,the remaining 24 bytes are read to complete the 26-byte frame.
    # Once complete, the protocol decodes it.
    elif state == "READ":
        frame_buf.append(b)
        if len(frame_buf) == FRAME_LEN:
            complete_frames += 1
            buf = frame_buf # renamed for clarity

            # --------------------------------------------------------
            # FIELD EXTRACTION (according to protocol byte layout)
            # --------------------------------------------------------
            # The protocol defines fixed byte offsets for each field.
            # These offsets are stable and must not be altered.

            seq = buf[5]    # Sequence number
            ptx = buf[7]    # PTX field (must be ASCII 'P')
            ttx = buf[16]   # TTX field (must be ASCII 'T')

            # RPM and VLT are big-endian 16-bit unsigned integers.
            rpm = (buf[8] << 8) | buf[9]
            vlt = (buf[10] << 8) | buf[11]

            # CRT is a 16-bit signed integer in little-endian format.
            crt = (buf[13] << 8) | buf[12]
            if crt > 32767:
                crt -= 65536    # converted to signed

            # Temperature bytes (MOS and CAP)
            mos_raw = buf[14]
            cap_raw = buf[15]

            # Lookup temperatures in the protocol-defined table.
            # Missing entries constitute corruption.
            mos_tmp = temperature_table.get(mos_raw)
            cap_tmp = temperature_table.get(cap_raw)

            # --------------------------------------------------------
            # TIMESTAMP ASSEMBLY (8-byte big-endian)
            # --------------------------------------------------------
            # The timestamp spans bytes 17–24 and represents microseconds since the Unix epoch. 
            # It is assembled manually to avoid endianness ambiguity.
            ts_raw = 0
            for i in range(17, 25):
                ts_raw = (ts_raw << 8) | buf[i]
            timestamps.append(ts_raw)
            ts_iso = ts_to_iso(ts_raw)

            # --------------------------------------------------------
            # CHECKSUM CALCULATION
            # --------------------------------------------------------
            # The checksum covers the first 25 bytes of the frame.
            # Protocol definition: checksum = 255 - (sum(bytes[0:25]) % 256)
            checksum_sum = sum(buf[:25])
            checksum_calc = (255 - (checksum_sum % 256)) % 256
            checksum_recv = buf[25]
            checksum_ok = (checksum_calc == checksum_recv)

            # --------------------------------------------------------
            # WRITING TO THE OUTPUT CSV (ALL FRAMES)
            # --------------------------------------------------------
            # This CSV contains every frame, regardless of corruption.
            fout.write(
                f"~~,{buf[2]},{buf[3]},{buf[4]},{seq},{buf[6]},"
                f"{printable_char(ptx)},{rpm},{vlt},{crt},"
                f"{(mos_tmp if mos_tmp is not None else 0):.1f},"
                f"{(cap_tmp if cap_tmp is not None else 0):.1f},"
                f"{printable_char(ttx)},{ts_raw},{checksum_recv},\n"
            )

            # --------------------------------------------------------
            # CORRUPTION CLASSIFICATION
            # --------------------------------------------------------
            # The definition of dataframe corruption is as follows:
            #   • PTX mismatch
            #   • TTX mismatch
            #   • Invalid temperature lookup
            #   • Checksum failure
            #   • Incomplete frame
            issues = []
            if ptx != 0x50:
                issues.append("P_mismatch") # PTX must be ASCII 'P' (0x50)
            if ttx != 0x54:
                issues.append("T_mismatch") # TTX must be ASCII 'T' (0x54)
            if mos_tmp is None or cap_tmp is None:
                issues.append("Invalid_temp") # Temperature lookup must succeed for both MOS and CAP
            if not checksum_ok:
                issues.append("checksum_fail") # Checksum must match

            # --------------------------------------------------------
            # WRITING TO THE DETAILED CORRUPT FRAMES LOG CSV (COMPLETE FRAMES)
            # --------------------------------------------------------
            corrupted = len(issues) > 0
            if corrupted:
                corrupt_frames += 1
                frame_hex = frame_to_hex(buf)
                ptx_hex = f"0x{ptx:02X}"
                ttx_hex = f"0x{ttx:02X}"
                ptx_char_det = chr(ptx) if 32 <= ptx <= 126 else '.' # If PTX Byte printable hex code, use it, else use . as a placeholder
                ttx_char_det = chr(ttx) if 32 <= ttx <= 126 else '.' # If TTX Byte printable hex code, use it, else use . as a placeholder
                checksum_calc_hex = f"0x{checksum_calc:02X}" # Hex code of checksum calculated from bytes 1-25
                checksum_recv_hex = f"0x{checksum_recv:02X}" # Hex code of checksum received from byte 26
                mos_tmp_str = f"{mos_tmp:.1f}" if mos_tmp is not None else "" # MOS TMP integrity
                cap_tmp_str = f"{cap_tmp:.1f}" if cap_tmp is not None else "" # CAP TMP intergrity
                issues_str = ';'.join(issues)
                byte_dump = ';'.join(f"{i:02d}:{buf[i]:02X}" for i in range(len(buf)))

                fd.write(
                    f"{seq},{frame_index},True,{frame_hex},{ptx_hex},{ptx_char_det},{ttx_hex},{ttx_char_det},"
                    f"{checksum_calc_hex},{checksum_recv_hex},{mos_raw},{cap_raw},{mos_tmp_str},{cap_tmp_str},{ts_raw},{ts_iso},\"{issues_str}\"\n"
                )
                
                # ----------------------------------------------------
                # STORING CORRUPTION METADATA FOR FINAL SUMMARY
                # ----------------------------------------------------
                corrupted_records.append({
                    "seq": seq,
                    "frame_index": frame_index,
                    "frame_hex": frame_hex,
                    "issues": issues,
                    "checksum_ok": checksum_ok,
                    "checksum_calc": checksum_calc,
                    "checksum_recv": checksum_recv,
                    "ptx": ptx,
                    "ttx": ttx,
                    "rpm": rpm,
                    "vlt": vlt,
                    "crt": crt,
                    "mos_raw": mos_raw,
                    "cap_raw": cap_raw,
                    "mos_tmp": mos_tmp,
                    "cap_tmp": cap_tmp,
                    "ts_raw": ts_raw,
                    "ts_iso": ts_iso,
                })
                
            # --------------------------------------------------------
            # RESETTING STATE MACHINE FOR THE NEXT FRAME
            # --------------------------------------------------------
            frame_index += 1
            state = "SEEK"
            frame_buf = []

    # Next Byte is read        
    byte = fin.read(1)

# --------------------------------------------------------
# HANDLING OF INCOMPLETE FRAMES AND WRITING TO THE DETAILED CORRUPT FRAMES LOG CSV
# --------------------------------------------------------
if frame_buf and 0 < len(frame_buf) < FRAME_LEN:
    seq = frame_buf[5] if len(frame_buf) > 5 else -1
    frame_hex = frame_to_hex(frame_buf)
    fd.write(f"{seq},{frame_index},False,{frame_hex},,,,"
             f",,,,{''},{''},{''},{''},\"incomplete_frame\"\n")
    incomplete_frames += 1
    corrupted_records.append({
        "seq": seq,
        "frame_index": frame_index,
        "frame_hex": frame_hex,
        "issues": ["incomplete_frame"],
        "checksum_ok": False,
        "checksum_calc": None,
        "checksum_recv": None,
        "ptx": None,
        "ttx": None,
        "rpm": None,
        "vlt": None,
        "crt": None,
        "mos_raw": None,
        "cap_raw": None,
        "mos_tmp": None,
        "cap_tmp": None,
        "ts_raw": None,
        "ts_iso": None,
    })
# --------------------------------------------------------
# REACHING THE END OF THE .BIN FILE AND WRTING THE COUNT OF COMPLETE FRAMES, CORRUPTED FRAMES AND INCOMPLETE FRAMES
# --------------------------------------------------------
print("End of file reached")
print(f"Total complete frames: {complete_frames}")
print(f"Total corrupted frames: {corrupt_frames}")
print(f"Total incomplete frames: {incomplete_frames}")


# ------------------------------------------------------------
# TIMESTAMP HANDLING AND SUMMARY
# ------------------------------------------------------------
# This block extracts the first timestamp and prints:
#   • Raw timestamp integer
#   • Interpreted date (YYYY-MM-DD)
#   • Day, Month, Year fields
#   • A note indicating microsecond interpretation

if timestamps:
    first_ts = timestamps[0]
    try:
        dt = datetime.fromtimestamp(first_ts / 1_000_000.0, tz=timezone.utc)
        print(f"First timestamp (raw): {first_ts}")
        print(f"Date: {dt.year:04d}-{dt.month:02d}-{dt.day:02d}")
        print(f"Day: {dt.day}")
        print(f"Month: {dt.month}")
        print(f"Year: {dt.year}")
        print("(Interpreted as microseconds)")
    except Exception:
        # If conversion fails, the raw timestamp is still printed
        print(f"First timestamp (raw): {first_ts}")
        print("Date: invalid")
        print("(Interpreted as microseconds)")


# ------------------------------------------------------------
# CORRUPTED AND INCOMPLETE FRAMES SUMMARY
# ------------------------------------------------------------
# For each corrupted or incomplete frame, the following summary is printed:
#   • Sequence number
#   • Corruption / Incompletion reason(s)
#   • Full frame hex
#   • Decoded fields (RPM, VLT, CRT, temperatures)
#   • Timestamp raw + ISO
#   • Expected vs received values for PTX, TTX, checksum, temperatures
print("----------------------------------------")
print("Corrupted / Incomplete frames summary")
print("----------------------------------------")
print(f"Total frames processed: {complete_frames}")
print(f"Total corrupted frames: {corrupt_frames}")
print(f"Total incomplete frames: {incomplete_frames}")
print()

for rec in corrupted_records:
    seq = rec["seq"]
    issues = rec["issues"]
    frame_hex = rec["frame_hex"]

    # Header line for the corrupted frame
    print(f"Seq {seq} – {', '.join(issues)}")
    print(f"Full frame: {frame_hex}")

    # --------------------------------------------------------
    # DECODED FIELDS
    # --------------------------------------------------------
    print("Decoded fields:")
    if rec["rpm"] is not None:
        print(f"  RPM: {rec['rpm']}")
        print(f"  VLT: {rec['vlt']}")
        print(f"  CRT: {rec['crt']}")

        # MOS temperature
        if rec["mos_tmp"] is not None:
            print(f"  MOS temp: {rec['mos_tmp']:.1f}°C (raw 0x{rec['mos_raw']:02X})")
        else:
            print(f"  MOS temp: INVALID (raw 0x{rec['mos_raw']:02X})")

        # CAP temperature
        if rec["cap_tmp"] is not None:
            print(f"  CAP temp: {rec['cap_tmp']:.1f}°C (raw 0x{rec['cap_raw']:02X})")
        else:
            print(f"  CAP temp: INVALID (raw 0x{rec['cap_raw']:02X})")

        # Timestamp fields
        if rec["ts_raw"] is not None:
            print(f"  Timestamp raw: {rec['ts_raw']}")
            if rec["ts_iso"] is not None:
                print(f"  Timestamp ISO: {rec['ts_iso']}")
            else:
                print("  Timestamp ISO: invalid")
    else:
        # Incomplete frame with undecryptable fields
        print("  (Frame incomplete, fields not fully decoded)")

    # --------------------------------------------------------
    # EXPECTED VS RECEIVED VALUES
    # --------------------------------------------------------

    # PTX mismatch
    if "P_mismatch" in issues and rec["ptx"] is not None:
        print(f"Expected PTX: 'P' (0x50)")
        print(f"Got PTX: {printable_char(rec['ptx'])} (0x{rec['ptx']:02X})")

    # TTX mismatch    
    if "T_mismatch" in issues and rec["ttx"] is not None:
        print(f"Expected TTX: 'T' (0x54)")
        print(f"Got TTX: {printable_char(rec['ttx'])} (0x{rec['ttx']:02X})")

    # Checksum mismatch    
    if "checksum_fail" in issues and rec["checksum_calc"] is not None:
        print(f"Checksum expected: 0x{rec['checksum_calc']:02X}")
        print(f"Checksum received: 0x{rec['checksum_recv']:02X}")

    # Invalid temperature lookup   
    if "Invalid_temp" in issues:
        if rec["mos_tmp"] is None and rec["mos_raw"] is not None:
            print(f"Expected MOS temp: valid lookup entry")
            print(f"Got MOS temp raw: 0x{rec['mos_raw']:02X} (not in table)")
        if rec["cap_tmp"] is None and rec["cap_raw"] is not None:
            print(f"Expected CAP temp: valid lookup entry")
            print(f"Got CAP temp raw: 0x{rec['cap_raw']:02X} (not in table)")

    # Incomplete frame        
    if "incomplete_frame" in issues:
        print("Frame was incomplete; expected 26 bytes, got fewer.")
    print() # blank line between corrupted/incomplete frames


# ------------------------------------------------------------
# FILE CLOSING
# ------------------------------------------------------------
fin.close()
fout.close()
fd.close()
