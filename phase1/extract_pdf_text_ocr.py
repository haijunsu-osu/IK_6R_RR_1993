#!/usr/bin/env python3
"""
Phase 1: Extract text/equation artifacts from reference PDFs.

Pipeline:
1. Try direct PDF text extraction with pypdf.
2. If a page has too little text, attempt OCR fallback (pytesseract + pdf2image).
3. Save per-paper markdown + JSON trace artifacts.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional

from pypdf import PdfReader


EQ_HINT_RE = re.compile(
    r"(=|det|tan|sin|cos|polynomial|eigen|resultant|matrix|x[1-6]|s[1-6]|c[1-6])",
    re.IGNORECASE,
)


def _normalize_text(s: str) -> str:
    s = s.replace("\r\n", "\n").replace("\r", "\n")
    s = s.replace("\u00ad", "")
    s = s.replace("\u2013", "-").replace("\u2014", "-")
    s = re.sub(r"[ \t]+", " ", s)
    s = re.sub(r"\n{3,}", "\n\n", s)
    return s.strip()


def _equation_candidates(text: str) -> List[str]:
    out: List[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if len(line) > 220:
            continue
        if EQ_HINT_RE.search(line):
            out.append(line)
            continue
        ops = sum(ch in "=+-*/()[]{}^" for ch in line)
        if ops >= 3:
            out.append(line)
    return out


def _safe_import_ocr():
    try:
        import pytesseract  # type: ignore
        from pdf2image import convert_from_path  # type: ignore
    except Exception:
        return None, None
    return pytesseract, convert_from_path


def _load_sidecar_ocr_pages(pdf_stem: str) -> dict[int, str]:
    candidates = [
        Path("phase1/sidecar_ocr") / f"{pdf_stem}_ocr.txt",
        Path("../utilities") / f"{pdf_stem}_ocr.txt",
    ]
    sidecar = next((p for p in candidates if p.exists()), None)
    if sidecar is None:
        return {}

    lines = sidecar.read_text(encoding="utf-8", errors="ignore").splitlines()
    pages: dict[int, List[str]] = {}
    current_page: Optional[int] = None
    header_re = re.compile(r"^=+\s*PAGE\s+(\d+)\s*=+$")
    for raw in lines:
        m = header_re.match(raw.strip())
        if m:
            current_page = int(m.group(1))
            pages[current_page] = []
            continue
        if current_page is not None:
            pages[current_page].append(raw)
    return {k: _normalize_text("\n".join(v)) for k, v in pages.items()}


@dataclass
class PageRecord:
    page: int
    method: str
    chars: int
    equation_lines: int


@dataclass
class PaperRecord:
    pdf: str
    pages: int
    ocr_pages: int
    direct_pages: int
    total_chars: int
    total_equation_lines: int
    markdown_artifact: str
    equations_artifact: str


def extract_pdf(pdf_path: Path, out_dir: Path, min_direct_chars: int, ocr_dpi: int) -> PaperRecord:
    reader = PdfReader(str(pdf_path))
    pytesseract, convert_from_path = _safe_import_ocr()
    ocr_available = pytesseract is not None and convert_from_path is not None
    sidecar_ocr_pages = _load_sidecar_ocr_pages(pdf_path.stem)

    stem = pdf_path.stem
    md_out = out_dir / f"{stem}_extracted.md"
    eq_out = out_dir / f"{stem}_equations.md"
    page_trace_out = out_dir / f"{stem}_page_trace.json"

    page_records: List[PageRecord] = []
    md_lines: List[str] = [f"# Extracted Text: {pdf_path.name}", ""]
    eq_lines: List[str] = [f"# Equation Candidates: {pdf_path.name}", ""]

    for i, page in enumerate(reader.pages, start=1):
        extracted = _normalize_text(page.extract_text() or "")
        method = "direct"
        text = extracted

        if len(extracted) < min_direct_chars and ocr_available:
            try:
                images = convert_from_path(
                    str(pdf_path),
                    dpi=ocr_dpi,
                    first_page=i,
                    last_page=i,
                )
                if images:
                    ocr_text = _normalize_text(pytesseract.image_to_string(images[0], lang="eng"))
                    if len(ocr_text) > len(extracted):
                        text = ocr_text
                        method = "ocr"
            except Exception:
                # Keep direct extraction when OCR fails.
                pass
        if len(text) < min_direct_chars and i in sidecar_ocr_pages:
            sidecar_text = sidecar_ocr_pages[i]
            if len(sidecar_text) > len(text):
                text = sidecar_text
                method = "ocr_sidecar"

        eqs = _equation_candidates(text)

        md_lines.extend(
            [
                f"## Page {i}",
                "",
                f"- extraction_method: `{method}`",
                f"- char_count: `{len(text)}`",
                "",
                "```text",
                text if text else "[no text extracted]",
                "```",
                "",
            ]
        )
        if eqs:
            eq_lines.append(f"## Page {i}")
            eq_lines.append("")
            for e in eqs:
                eq_lines.append(f"- {e}")
            eq_lines.append("")

        page_records.append(
            PageRecord(
                page=i,
                method=method,
                chars=len(text),
                equation_lines=len(eqs),
            )
        )

    md_out.write_text("\n".join(md_lines), encoding="utf-8")
    eq_out.write_text("\n".join(eq_lines), encoding="utf-8")
    page_trace_out.write_text(
        json.dumps([asdict(r) for r in page_records], indent=2),
        encoding="utf-8",
    )

    ocr_pages = sum(1 for r in page_records if r.method.startswith("ocr"))
    direct_pages = len(page_records) - ocr_pages
    total_chars = sum(r.chars for r in page_records)
    total_eq = sum(r.equation_lines for r in page_records)

    return PaperRecord(
        pdf=pdf_path.name,
        pages=len(page_records),
        ocr_pages=ocr_pages,
        direct_pages=direct_pages,
        total_chars=total_chars,
        total_equation_lines=total_eq,
        markdown_artifact=str(md_out),
        equations_artifact=str(eq_out),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Extract PDF text with OCR fallback.")
    parser.add_argument(
        "--pdfs",
        nargs="+",
        default=[
            "RaghavanRoth6R1993.pdf",
            "MCtra94.pdf",
            "A_Robot_Manipulator_With_16_Real_Inverse_Kinematic.pdf",
        ],
        help="List of input PDF paths.",
    )
    parser.add_argument(
        "--out-dir",
        default="phase1/artifacts",
        help="Artifact output directory.",
    )
    parser.add_argument(
        "--min-direct-chars",
        type=int,
        default=120,
        help="Use OCR fallback if direct extraction chars on a page are below this threshold.",
    )
    parser.add_argument(
        "--ocr-dpi",
        type=int,
        default=300,
        help="OCR render DPI when fallback is used.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    summaries: List[PaperRecord] = []
    for pdf in args.pdfs:
        pdf_path = Path(pdf)
        if not pdf_path.exists():
            print(f"[warn] missing PDF: {pdf_path}", file=sys.stderr)
            continue
        summary = extract_pdf(
            pdf_path=pdf_path,
            out_dir=out_dir,
            min_direct_chars=args.min_direct_chars,
            ocr_dpi=args.ocr_dpi,
        )
        summaries.append(summary)
        print(
            f"[ok] {summary.pdf}: pages={summary.pages}, direct={summary.direct_pages}, "
            f"ocr={summary.ocr_pages}, eq_lines={summary.total_equation_lines}"
        )

    summary_path = out_dir / "phase1_extraction_summary.json"
    summary_path.write_text(
        json.dumps([asdict(s) for s in summaries], indent=2),
        encoding="utf-8",
    )
    print(f"[done] wrote summary: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
