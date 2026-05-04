"""
Cross-layer contract parsers.

Extracts interface contracts (opcodes, bit widths, reset defaults, packet
layouts) directly from the source files of each layer:
  - Python GUI:  radar_protocol.py
  - FPGA RTL:    radar_system_top.v, usb_data_interface_ft2232h.v,
                 usb_data_interface.v
  - STM32 MCU:   RadarSettings.cpp, main.cpp

These parsers do NOT define the expected values — they discover what each
layer actually implements, so the test can compare layers against ground
truth and find bugs where both sides are wrong (like the 0x06 phantom
opcode or the status_words[0] 37-bit truncation).
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

# ---------------------------------------------------------------------------
# Repository layout (relative to repo root)
# ---------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parents[3]
GUI_DIR = REPO_ROOT / "9_Firmware" / "9_3_GUI"
FPGA_DIR = REPO_ROOT / "9_Firmware" / "9_2_FPGA"
MCU_DIR = REPO_ROOT / "9_Firmware" / "9_1_Microcontroller"
MCU_LIB_DIR = MCU_DIR / "9_1_1_C_Cpp_Libraries"
MCU_CODE_DIR = MCU_DIR / "9_1_3_C_Cpp_Code"
XDC_DIR = FPGA_DIR / "constraints"


# ===================================================================
# Data structures
# ===================================================================

@dataclass
class OpcodeEntry:
    """One opcode as declared in a single layer."""
    name: str
    value: int
    register: str = ""      # Verilog register name it writes to
    bit_slice: str = ""     # e.g. "[3:0]", "[15:0]", "[0]"
    bit_width: int = 0      # derived from bit_slice
    reset_default: int | None = None
    is_pulse: bool = False  # True for trigger/request opcodes


@dataclass
class StatusWordField:
    """One field inside a status_words[] entry."""
    name: str
    word_index: int
    msb: int   # bit position in the 32-bit word (0-indexed from LSB)
    lsb: int
    width: int


@dataclass
class DataPacketField:
    """One field in the 11-byte data packet."""
    name: str
    byte_start: int   # first byte index (0 = header)
    byte_end: int     # last byte index (inclusive)
    width_bits: int


@dataclass
class PacketConstants:
    """Header/footer/size constants for a packet type."""
    header: int
    footer: int
    size: int


@dataclass
class SettingsField:
    """One field in the STM32 SET...END settings packet."""
    name: str
    offset: int        # byte offset from start of payload (after "SET")
    size: int          # bytes
    c_type: str        # "double" or "uint32_t"


@dataclass
class GpioPin:
    """A GPIO pin with direction."""
    name: str
    pin_id: str        # e.g. "PD8", "H11"
    direction: str     # "output" or "input"
    layer: str         # "stm32" or "fpga"


@dataclass
class ConcatWidth:
    """Result of counting bits in a Verilog concatenation."""
    total_bits: int
    target_bits: int   # width of the register being assigned to
    fragments: list[tuple[str, int]] = field(default_factory=list)
    truncated: bool = False


# ===================================================================
# Python layer parser
# ===================================================================

def parse_python_opcodes(filepath: Path | None = None) -> dict[int, OpcodeEntry]:
    """Parse the Opcode enum from radar_protocol.py.
    Returns {opcode_value: OpcodeEntry}.
    """
    if filepath is None:
        filepath = GUI_DIR / "radar_protocol.py"
    text = filepath.read_text()

    # Find the Opcode class body
    match = re.search(r'class Opcode\b.*?(?=\nclass |\Z)', text, re.DOTALL)
    if not match:
        raise ValueError(f"Could not find 'class Opcode' in {filepath}")

    opcodes: dict[int, OpcodeEntry] = {}
    for m in re.finditer(r'(\w+)\s*=\s*(0x[0-9a-fA-F]+)', match.group()):
        name = m.group(1)
        value = int(m.group(2), 16)
        opcodes[value] = OpcodeEntry(name=name, value=value)
    return opcodes


def parse_python_packet_constants(filepath: Path | None = None) -> dict[str, PacketConstants]:
    """
    Extract HEADER_BYTE, FOOTER_BYTE, STATUS_HEADER_BYTE, STATUS_PACKET_SIZE.

    Note on the data packet: PR-G replaced the fixed 11-byte v1 frame with a
    variable-length bulk frame (header + optional sections + footer), so a
    single ``DATA_PACKET_SIZE`` constant no longer characterizes the data
    layer. Python keeps ``DATA_PACKET_SIZE = 11`` as a back-compat alias for
    legacy FT601 log files; we deliberately do NOT cross-check it against
    the FPGA, which has no equivalent localparam in either USB module.
    """
    if filepath is None:
        filepath = GUI_DIR / "radar_protocol.py"
    text = filepath.read_text()

    def _find(pattern: str) -> int:
        m = re.search(pattern, text)
        if not m:
            raise ValueError(f"Pattern not found: {pattern}")
        val = m.group(1)
        return int(val, 16) if val.startswith("0x") else int(val)

    footer = _find(r'FOOTER_BYTE\s*=\s*(0x[0-9a-fA-F]+|\d+)')
    status_header = _find(r'STATUS_HEADER_BYTE\s*=\s*(0x[0-9a-fA-F]+|\d+)')
    status_size = _find(r'STATUS_PACKET_SIZE\s*=\s*(\d+)')

    return {
        "status": PacketConstants(header=status_header, footer=footer, size=status_size),
    }


def parse_python_data_packet_fields(filepath: Path | None = None) -> list[DataPacketField]:
    """
    Extract byte offsets from parse_data_packet() by finding struct.unpack_from calls.
    Returns fields in byte order.
    """
    if filepath is None:
        filepath = GUI_DIR / "radar_protocol.py"
    text = filepath.read_text()

    # Find parse_data_packet method body
    match = re.search(
        r'def parse_data_packet\(.*?\).*?(?=\n    @|\n    def |\nclass |\Z)',
        text, re.DOTALL
    )
    if not match:
        raise ValueError("Could not find parse_data_packet()")

    body = match.group()
    fields: list[DataPacketField] = []

    # Match patterns like: range_q = _to_signed16(struct.unpack_from(">H", raw, 1)[0])
    for m in re.finditer(
        r'(\w+)\s*=\s*_to_signed16\(struct\.unpack_from\("(>[HIBhib])", raw, (\d+)\)',
        body
    ):
        name = m.group(1)
        fmt = m.group(2)
        offset = int(m.group(3))
        fmt_char = fmt[-1].upper()
        size = {"H": 2, "I": 4, "B": 1}[fmt_char]
        fields.append(DataPacketField(
            name=name, byte_start=offset,
            byte_end=offset + size - 1,
            width_bits=size * 8
        ))

    # Match detection = raw[9] & 0x01  (direct access)
    for m in re.finditer(r'(\w+)\s*=\s*raw\[(\d+)\]\s*&\s*(0x[0-9a-fA-F]+|\d+)', body):
        name = m.group(1)
        offset = int(m.group(2))
        fields.append(DataPacketField(
            name=name, byte_start=offset, byte_end=offset, width_bits=1
        ))

    # Match intermediate variable pattern: var = raw[N], then field = var & MASK
    for m in re.finditer(r'(\w+)\s*=\s*raw\[(\d+)\]', body):
        var_name = m.group(1)
        offset = int(m.group(2))
        # Find fields derived from this intermediate variable
        for m2 in re.finditer(
            rf'(\w+)\s*=\s*(?:\({var_name}\s*>>\s*\d+\)\s*&|{var_name}\s*&)\s*'
            r'(0x[0-9a-fA-F]+|\d+)',
            body,
        ):
            name = m2.group(1)
            # Skip if already captured by direct raw[] access pattern
            if not any(f.name == name for f in fields):
                fields.append(DataPacketField(
                    name=name, byte_start=offset, byte_end=offset,
                    width_bits=1
                ))

    fields.sort(key=lambda f: f.byte_start)
    return fields


def parse_python_status_fields(filepath: Path | None = None) -> list[StatusWordField]:
    """
    Extract bit shift/mask operations from parse_status_packet().
    Returns the fields with word index and bit positions as Python sees them.
    """
    if filepath is None:
        filepath = GUI_DIR / "radar_protocol.py"
    text = filepath.read_text()

    match = re.search(
        r'def parse_status_packet\(.*?\).*?(?=\n    @|\n    def |\nclass |\Z)',
        text, re.DOTALL
    )
    if not match:
        raise ValueError("Could not find parse_status_packet()")

    body = match.group()
    fields: list[StatusWordField] = []

    # Pattern: sr.field = (words[N] >> S) & MASK  # noqa: ERA001
    for m in re.finditer(
        r'sr\.(\w+)\s*=\s*\(words\[(\d+)\]\s*>>\s*(\d+)\)\s*&\s*(0x[0-9a-fA-F]+|\d+)',
        body
    ):
        name = m.group(1)
        word_idx = int(m.group(2))
        shift = int(m.group(3))
        mask_str = m.group(4)
        mask = int(mask_str, 16) if mask_str.startswith("0x") else int(mask_str)
        width = mask.bit_length()
        fields.append(StatusWordField(
            name=name, word_index=word_idx,
            msb=shift + width - 1, lsb=shift, width=width
        ))

    # Pattern: sr.field = words[N] & MASK  (no shift)
    for m in re.finditer(
        r'sr\.(\w+)\s*=\s*words\[(\d+)\]\s*&\s*(0x[0-9a-fA-F]+|\d+)',
        body
    ):
        name = m.group(1)
        word_idx = int(m.group(2))
        mask_str = m.group(3)
        mask = int(mask_str, 16) if mask_str.startswith("0x") else int(mask_str)
        width = mask.bit_length()
        # Skip if already captured by the shift pattern
        if not any(f.name == name and f.word_index == word_idx for f in fields):
            fields.append(StatusWordField(
                name=name, word_index=word_idx,
                msb=width - 1, lsb=0, width=width
            ))

    return fields


# ===================================================================
# Verilog layer parser
# ===================================================================

def _parse_bit_slice(s: str) -> int:
    """Parse '[15:0]' -> 16, '[0]' -> 1, '' -> 16 (full cmd_value)."""
    m = re.match(r'\[(\d+):(\d+)\]', s)
    if m:
        return int(m.group(1)) - int(m.group(2)) + 1
    m = re.match(r'\[(\d+)\]', s)
    if m:
        return 1
    return 16  # default: full 16-bit cmd_value


def parse_verilog_opcodes(filepath: Path | None = None) -> dict[int, OpcodeEntry]:
    """
    Parse the opcode case statement from radar_system_top.v.
    Returns {opcode_value: OpcodeEntry}.
    """
    if filepath is None:
        filepath = FPGA_DIR / "radar_system_top.v"
    text = filepath.read_text()

    # Find the command decode case block
    # Pattern: case statement with 8'hXX opcodes
    opcodes: dict[int, OpcodeEntry] = {}

    # Pattern 1: Simple assignment — 8'hXX: register <= rhs;
    for m in re.finditer(
        r"8'h([0-9a-fA-F]{2})\s*:\s*(\w+)\s*<=\s*(.*?)(?:;|$)",
        text, re.MULTILINE
    ):
        value = int(m.group(1), 16)
        register = m.group(2)
        rhs = m.group(3).strip()

        # Determine if it's a pulse (assigned literal 1)
        is_pulse = rhs in ("1", "1'b1")

        # Extract bit slice from the RHS (e.g., usb_cmd_value[3:0])
        bit_slice = ""
        slice_m = re.search(r'usb_cmd_value(\[\d+(?::\d+)?\])', rhs)
        if slice_m:
            bit_slice = slice_m.group(1)
        elif "usb_cmd_value" in rhs:
            bit_slice = "[15:0]"  # full width

        bit_width = _parse_bit_slice(bit_slice) if bit_slice else 0

        opcodes[value] = OpcodeEntry(
            name=register,
            value=value,
            register=register,
            bit_slice=bit_slice,
            bit_width=bit_width,
            is_pulse=is_pulse,
        )

    # Pattern 2: begin...end blocks — 8'hXX: begin ... register <= ... end
    # These are used for opcodes with validation logic (e.g., 0x15 clamp)
    for m in re.finditer(
        r"8'h([0-9a-fA-F]{2})\s*:\s*begin\b(.*?)end\b",
        text, re.DOTALL
    ):
        value = int(m.group(1), 16)
        if value in opcodes:
            continue  # Already captured by pattern 1
        body = m.group(2)

        # Find the first register assignment (host_xxx <=)
        assign_m = re.search(r'(host_\w+)\s*<=\s*(.+?);', body)
        if not assign_m:
            continue

        register = assign_m.group(1)
        rhs = assign_m.group(2).strip()

        bit_slice = ""
        slice_m = re.search(r'usb_cmd_value(\[\d+(?::\d+)?\])', body)
        if slice_m:
            bit_slice = slice_m.group(1)
        elif "usb_cmd_value" in body:
            bit_slice = "[15:0]"

        bit_width = _parse_bit_slice(bit_slice) if bit_slice else 0

        opcodes[value] = OpcodeEntry(
            name=register,
            value=value,
            register=register,
            bit_slice=bit_slice,
            bit_width=bit_width,
            is_pulse=False,
        )

    return opcodes


def _resolve_verilog_value(rhs: str, macros: dict[str, int]) -> int | None:
    """
    Resolve a Verilog RHS expression to an integer. Supports:
      * Plain decimal:           ``1234``
      * Verilog literal:         ``16'd1234``, ``8'hAA``, ``2'b01``, ``6'b000_111``
      * Macro reference:         ```MACRO_NAME``  (looked up in *macros*)
      * Width-prefixed macro:    ``16'd`MACRO_NAME``
    Returns None when the RHS can't be resolved (e.g. RHS is a wire name,
    concatenation, or an undefined macro).
    """
    rhs = rhs.strip()

    # Width-prefixed form: "16'd...", "8'h...", "2'b...", "4'o..."
    width_match = re.match(r"^(\d+)'([bdho])(.*)$", rhs)
    if width_match:
        base_char = width_match.group(2)
        rest = width_match.group(3).strip()
        # Width-prefixed macro reference: 16'd`RP_DEF_FOO
        if rest.startswith("`"):
            return macros.get(rest[1:].strip())
        digits = rest.replace("_", "")
        if not re.fullmatch(r"[0-9a-fA-F]+", digits):
            return None
        base = {"b": 2, "d": 10, "h": 16, "o": 8}[base_char]
        try:
            return int(digits, base)
        except ValueError:
            return None

    # Bare macro reference: `MACRO_NAME
    if rhs.startswith("`"):
        return macros.get(rhs[1:].strip())

    # Plain decimal
    if rhs.isdigit():
        return int(rhs)

    return None


def parse_radar_params_macros(filepath: Path | None = None) -> dict[str, int]:
    """
    Parse `define directives in radar_params.vh into a name → integer map.
    Resolves up to two macro→macro indirections (none expected today, kept
    for forward compatibility).
    """
    if filepath is None:
        filepath = FPGA_DIR / "radar_params.vh"
    text = filepath.read_text()

    raw: dict[str, str] = {}
    for line in text.splitlines():
        m = re.match(r"^\s*`define\s+(\w+)\s+(\S.*?)(?:\s*//.*)?\s*$", line)
        if m:
            raw[m.group(1)] = m.group(2).strip()

    macros: dict[str, int] = {}
    for _ in range(3):  # bounded fixed-point — file is small, runs in microseconds
        progressed = False
        for name, rhs in raw.items():
            if name in macros:
                continue
            val = _resolve_verilog_value(rhs, macros)
            if val is not None:
                macros[name] = val
                progressed = True
        if not progressed:
            break
    return macros


def parse_verilog_reset_defaults(filepath: Path | None = None) -> dict[str, int]:
    """
    Parse the reset block from radar_system_top.v.
    Returns {register_name: reset_value}.

    Expands ```RP_DEF_*`` style macro references against radar_params.vh so
    that fields like ``host_long_chirp_cycles <= 16'd`RP_DEF_LONG_CHIRP_CYCLES``
    resolve to integers instead of being silently dropped.
    """
    if filepath is None:
        filepath = FPGA_DIR / "radar_system_top.v"
    text = filepath.read_text()
    macros = parse_radar_params_macros()

    defaults: dict[str, int] = {}

    # Capture every "host_X <= <expr>;" assignment, regardless of RHS form.
    # Resolution to an integer happens via _resolve_verilog_value, which
    # rejects (returns None for) RHSes that aren't statically known
    # constants (e.g. concatenations or wire names from the opcode decode
    # block — those land below the reset block, and we keep only the first
    # occurrence anyway).
    for m in re.finditer(r'(host_\w+)\s*<=\s*([^;]+?)\s*;', text):
        reg = m.group(1)
        if reg in defaults:
            continue  # reset block precedes opcode block; first wins
        val = _resolve_verilog_value(m.group(2), macros)
        if val is not None:
            defaults[reg] = val

    return defaults


def parse_verilog_register_widths(filepath: Path | None = None) -> dict[str, int]:
    """
    Parse register declarations from radar_system_top.v.
    Returns {register_name: bit_width}.
    """
    if filepath is None:
        filepath = FPGA_DIR / "radar_system_top.v"
    text = filepath.read_text()

    widths: dict[str, int] = {}

    # Match: reg [15:0] host_detect_threshold;
    # Also:  reg        host_trigger_pulse;
    for m in re.finditer(
        r'reg\s+(?:\[\s*(\d+)\s*:\s*(\d+)\s*\]\s+)?(host_\w+)\s*;',
        text
    ):
        width = int(m.group(1)) - int(m.group(2)) + 1 if m.group(1) is not None else 1
        widths[m.group(3)] = width

    return widths


def parse_verilog_packet_constants(
    filepath: Path | None = None,
) -> dict[str, PacketConstants]:
    """
    Extract HEADER, FOOTER, STATUS_HEADER, STATUS_PKT_LEN localparams.

    Note: ``DATA_PKT_LEN`` was retired in PR-G when the data path moved to a
    variable-length bulk frame (9-byte header + optional sections + 1-byte
    footer). There is no equivalent constant in the v2 module; the data
    layer is exercised separately by `parse_verilog_data_mux()` against the
    fixed 9-byte header section.
    """
    if filepath is None:
        filepath = FPGA_DIR / "usb_data_interface_ft2232h.v"
    text = filepath.read_text()

    def _find(pattern: str) -> int:
        m = re.search(pattern, text)
        if not m:
            raise ValueError(f"Pattern not found in {filepath}: {pattern}")
        val = m.group(1)
        # Parse Verilog literals: 8'hAA → 0xAA, 5'd11 → 11
        vlog_m = re.match(r"\d+'h([0-9a-fA-F]+)", val)
        if vlog_m:
            return int(vlog_m.group(1), 16)
        vlog_m = re.match(r"\d+'d(\d+)", val)
        if vlog_m:
            return int(vlog_m.group(1))
        return int(val, 16) if val.startswith("0x") else int(val)

    footer_val = _find(r"localparam\s+FOOTER\s*=\s*(\d+'h[0-9a-fA-F]+)")
    status_hdr = _find(r"localparam\s+STATUS_HEADER\s*=\s*(\d+'h[0-9a-fA-F]+)")
    status_size = _find(r"STATUS_PKT_LEN\s*=\s*(\d+'d\d+)")

    return {
        "status": PacketConstants(header=status_hdr, footer=footer_val, size=status_size),
    }


def count_concat_bits(concat_expr: str, port_widths: dict[str, int]) -> ConcatWidth:
    """
    Count total bits in a Verilog concatenation expression like:
      {8'hFF, 3'b000, status_radar_mode, 5'b00000, status_stream_ctrl, status_cfar_threshold}

    Uses port_widths to resolve signal widths. Returns ConcatWidth.
    """
    # Remove outer braces
    inner = concat_expr.strip().strip("{}")
    fragments: list[tuple[str, int]] = []
    total = 0

    for part in re.split(r',\s*', inner):
        part = part.strip()
        if not part:
            continue

        # Literal: N'bXXX, N'dXXX, N'hXX, or just a decimal
        lit_match = re.match(r"(\d+)'[bdhoBDHO]", part)
        if lit_match:
            w = int(lit_match.group(1))
            fragments.append((part, w))
            total += w
            continue

        # Signal with bit select: sig[M:N] or sig[N]
        sel_match = re.match(r'(\w+)\[(\d+):(\d+)\]', part)
        if sel_match:
            w = int(sel_match.group(2)) - int(sel_match.group(3)) + 1
            fragments.append((part, w))
            total += w
            continue

        sel_match = re.match(r'(\w+)\[(\d+)\]', part)
        if sel_match:
            fragments.append((part, 1))
            total += 1
            continue

        # Bare signal: look up in port_widths
        if part in port_widths:
            w = port_widths[part]
            fragments.append((part, w))
            total += w
        else:
            # Unknown width — flag it
            fragments.append((part, -1))
            total = -1  # Can't compute
            break

    return ConcatWidth(
        total_bits=total,
        target_bits=32,
        fragments=fragments,
        truncated=total > 32 if total > 0 else False,
    )


def parse_verilog_status_word_concats(
    filepath: Path | None = None,
) -> dict[int, str]:
    """
    Extract the raw concatenation expression for each status_words[N] assignment.
    Returns {word_index: concat_expression_string}.
    """
    if filepath is None:
        filepath = FPGA_DIR / "usb_data_interface_ft2232h.v"
    text = filepath.read_text()

    results: dict[int, str] = {}

    # Multi-line concat: status_words[N] <= {... };
    # We need to handle multi-line expressions
    for m in re.finditer(
        r'status_words\[(\d+)\]\s*<=\s*(\{[^;]+\})\s*;',
        text, re.DOTALL
    ):
        idx = int(m.group(1))
        expr = m.group(2)
        # Strip single-line comments before normalizing whitespace
        expr = re.sub(r'//[^\n]*', '', expr)
        # Normalize whitespace
        expr = re.sub(r'\s+', ' ', expr).strip()
        results[idx] = expr

    return results


def get_usb_interface_port_widths(filepath: Path | None = None) -> dict[str, int]:
    """
    Parse port declarations from usb_data_interface_ft2232h.v module header.
    Returns {port_name: bit_width}.
    """
    if filepath is None:
        filepath = FPGA_DIR / "usb_data_interface_ft2232h.v"
    text = filepath.read_text()

    widths: dict[str, int] = {}

    # Match: input wire [15:0] status_cfar_threshold,
    # Also:  input wire        status_self_test_busy
    for m in re.finditer(
        r'(?:input|output)\s+(?:wire|reg)\s+(?:\[\s*(\d+)\s*:\s*(\d+)\s*\]\s+)?(\w+)',
        text
    ):
        width = int(m.group(1)) - int(m.group(2)) + 1 if m.group(1) is not None else 1
        widths[m.group(3)] = width

    return widths


def parse_verilog_data_mux(
    filepath: Path | None = None,
) -> list[DataPacketField]:
    """
    Parse the v2 data-frame 9-byte fixed header mux from
    usb_data_interface_ft2232h.v. Returns fields with byte positions and
    signal names.

    PR-G replaced the v1 11-byte fixed data packet (combinational
    ``always @(*) begin case (wr_byte_idx) ... data_pkt_byte = ...``) with
    a clocked FSM that emits a fixed 9-byte header followed by optional
    variable-length sections. This parser walks the WR_FRAME_HEADER
    ``case (wr_byte_idx[3:0]) ... ft_data_out <= ...`` block.
    """
    if filepath is None:
        filepath = FPGA_DIR / "usb_data_interface_ft2232h.v"
    text = filepath.read_text()

    # Find the WR_FRAME_HEADER mux: the case block that drives ft_data_out
    # from the low 4 bits of the byte index, one assignment per fixed-header
    # byte (4'd0..4'd8).
    match = re.search(
        r'case\s*\(\s*wr_byte_idx\s*\[\s*3\s*:\s*0\s*\]\s*\)(.*?)endcase',
        text, re.DOTALL
    )
    if not match:
        raise ValueError("Could not find v2 data-frame header mux")

    mux_body = match.group(1)
    entries: list[tuple[int, str]] = []

    for m in re.finditer(
        r"4'd(\d+)\s*:\s*ft_data_out\s*<=\s*(.+?);",
        mux_body, re.DOTALL
    ):
        idx = int(m.group(1))
        expr = m.group(2).strip()
        entries.append((idx, expr))

    # Helper: extract the dominant signal name from a mux expression.
    # Handles direct refs like ``range_profile_cap[31:24]``, ternaries
    # like ``stream_doppler_en ? doppler_real_cap[15:8] : 8'd0``, and
    # concat-ternaries like ``stream_cfar_en ? {…, cfar_detection_cap} : …``.
    def _extract_signal(expr: str) -> str | None:
        # If it's a ternary, use the true-branch to find the data signal
        tern = re.match(r'\w+\s*\?\s*(.+?)\s*:\s*.+', expr, re.DOTALL)
        target = tern.group(1) if tern else expr
        # Look for a known data signal (xxx_cap pattern or cfar_detection_cap)
        cap_match = re.search(r'(\w+_cap)\b', target)
        if cap_match:
            return cap_match.group(1)
        # Fall back to first identifier before a bit-select
        sig_match = re.match(r'(\w+?)(?:\[|$)', target)
        return sig_match.group(1) if sig_match else None

    # Group consecutive bytes by signal root name
    fields: list[DataPacketField] = []
    i = 0
    while i < len(entries):
        idx, expr = entries[i]
        if expr == "HEADER" or expr == "FOOTER":
            i += 1
            continue

        signal = _extract_signal(expr)
        if not signal:
            i += 1
            continue

        start_byte = idx
        end_byte = idx

        # Find consecutive bytes of the same signal
        j = i + 1
        while j < len(entries):
            _next_idx, next_expr = entries[j]
            next_sig = _extract_signal(next_expr)
            if next_sig == signal:
                end_byte = _next_idx
                j += 1
            else:
                break

        n_bytes = end_byte - start_byte + 1
        fields.append(DataPacketField(
            name=signal.replace("_cap", ""),
            byte_start=start_byte,
            byte_end=end_byte,
            width_bits=n_bytes * 8,
        ))
        i = j

    return fields


# ===================================================================
# STM32 / C layer parser
# ===================================================================

def parse_stm32_settings_fields(
    filepath: Path | None = None,
) -> list[SettingsField]:
    """
    Parse RadarSettings::parseFromUSB to extract field order, offsets, types.
    """
    if filepath is None:
        filepath = MCU_LIB_DIR / "RadarSettings.cpp"

    if not filepath.exists():
        return []  # MCU code not available (CI might not have it)

    text = filepath.read_text(encoding="latin-1")

    fields: list[SettingsField] = []

    # Look for memcpy + shift patterns that extract doubles and uint32s
    # Pattern for doubles: loop reading 8 bytes big-endian
    # Pattern for uint32: 4 bytes big-endian
    # We'll parse the assignment targets in order

    # Find the parseFromUSB function
    match = re.search(
        r'parseFromUSB\s*\(.*?\)\s*\{(.*?)^\}',
        text, re.DOTALL | re.MULTILINE
    )
    if not match:
        return fields

    body = match.group(1)

    # The fields are extracted sequentially from the payload.
    # Look for variable assignments that follow the memcpy/extraction pattern.
    # Based on known code: extractDouble / extractUint32 patterns
    field_names = [
        ("system_frequency", 8, "double"),
        ("chirp_duration_1", 8, "double"),
        ("chirp_duration_2", 8, "double"),
        ("chirps_per_position", 4, "uint32_t"),
        ("freq_min", 8, "double"),
        ("freq_max", 8, "double"),
        ("prf1", 8, "double"),
        ("prf2", 8, "double"),
        ("max_distance", 8, "double"),
        ("map_size", 8, "double"),
    ]

    offset = 0
    for name, size, ctype in field_names:
        # Verify the field name appears in the function body
        if name in body or name.replace("_", "") in body.lower():
            fields.append(SettingsField(
                name=name, offset=offset, size=size, c_type=ctype
            ))
        offset += size

    return fields


def parse_stm32_start_flag(
    filepath: Path | None = None,
) -> list[int]:
    """Parse the USB start flag bytes from USBHandler.cpp."""
    if filepath is None:
        filepath = MCU_LIB_DIR / "USBHandler.cpp"

    if not filepath.exists():
        return []

    text = filepath.read_text()

    # Look for the start flag array, e.g. {23, 46, 158, 237}
    match = re.search(r'start_flag.*?=\s*\{([^}]+)\}', text, re.DOTALL)
    if not match:
        # Try alternate patterns
        match = re.search(r'\{(\s*\d+\s*,\s*\d+\s*,\s*\d+\s*,\s*\d+\s*)\}', text)
        if not match:
            return []

    return [int(x.strip()) for x in match.group(1).split(",") if x.strip().isdigit()]


# ===================================================================
# GPIO parser
# ===================================================================

def parse_xdc_gpio_pins(filepath: Path | None = None) -> list[GpioPin]:
    """Parse XDC constraints for DIG_* pin assignments."""
    if filepath is None:
        filepath = XDC_DIR / "xc7a50t_ftg256.xdc"

    if not filepath.exists():
        return []

    text = filepath.read_text()
    pins: list[GpioPin] = []

    # Match: set_property PACKAGE_PIN XX [get_ports {signal_name}]
    for m in re.finditer(
        r'set_property\s+PACKAGE_PIN\s+(\w+)\s+\[get_ports\s+\{?(\w+)\}?\]',
        text
    ):
        pin = m.group(1)
        signal = m.group(2)
        if any(kw in signal for kw in ("stm32_", "reset_n", "dig_")):
            # Determine direction from signal name
            if signal in ("stm32_new_chirp", "stm32_new_elevation",
                         "stm32_new_azimuth", "stm32_mixers_enable"):
                direction = "input"  # FPGA receives these
            elif signal == "reset_n":
                direction = "input"
            else:
                direction = "unknown"
            pins.append(GpioPin(
                name=signal, pin_id=pin, direction=direction, layer="fpga"
            ))

    return pins


def parse_stm32_gpio_init(filepath: Path | None = None) -> list[GpioPin]:
    """Parse STM32 GPIO initialization for PD8-PD15 directions."""
    if filepath is None:
        filepath = MCU_CODE_DIR / "main.cpp"

    if not filepath.exists():
        return []

    text = filepath.read_text()
    pins: list[GpioPin] = []

    # Look for GPIO_InitStruct.Pin and GPIO_InitStruct.Mode patterns
    # This is approximate — STM32 HAL GPIO init is complex
    # Look for PD8-PD15 configuration (output vs input)

    # Pattern: GPIO_PIN_8 | GPIO_PIN_9 ... with Mode = OUTPUT
    # We'll find blocks that configure GPIOD pins
    for m in re.finditer(
        r'GPIO_InitStruct\.Pin\s*=\s*([^;]+);.*?'
        r'GPIO_InitStruct\.Mode\s*=\s*(\w+)',
        text, re.DOTALL
    ):
        pin_expr = m.group(1)
        mode = m.group(2)

        direction = "output" if "OUTPUT" in mode else "input"

        # Extract individual pin numbers
        for pin_m in re.finditer(r'GPIO_PIN_(\d+)', pin_expr):
            pin_num = int(pin_m.group(1))
            if 8 <= pin_num <= 15:
                pins.append(GpioPin(
                    name=f"PD{pin_num}",
                    pin_id=f"PD{pin_num}",
                    direction=direction,
                    layer="stm32"
                ))

    return pins
