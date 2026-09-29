"""Execute the actual Python/Rust documentation fences against this checkout.

Run from the repository root after installing the extension. Signature fragments
must use text fences. No code is copied into a separate example implementation.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import tempfile
import textwrap
from dataclasses import dataclass
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Snippet:
    path: Path
    line: int
    language: str
    code: str

    @property
    def label(self) -> str:
        return f"{self.path.relative_to(ROOT)}:{self.line}"


def snippets() -> list[Snippet]:
    """Read fenced examples, including indentation inside Zensical tabs."""
    result: list[Snippet] = []
    for path in [ROOT / "README.md", *sorted((ROOT / "docs").rglob("*.md"))]:
        language: str | None = None
        lines: list[str] = []
        start = 0
        for number, line in enumerate(path.read_text().splitlines(), 1):
            fence = re.fullmatch(r"\s*```(\w*)\s*", line)
            if fence:
                if language is None:
                    language = fence[1]
                    start = number + 1
                    lines = []
                else:
                    if language in {"python", "rust"}:
                        result.append(Snippet(path, start, language, textwrap.dedent("\n".join(lines))))
                    language = None
            elif language is not None:
                lines.append(line)
        if language is not None:
            raise ValueError(f"Unclosed fence in {path}:{start - 1}")
    return result


def check_python(snippet: Snippet) -> None:
    """Execute one standalone example; tracebacks retain its page/line location."""
    source = "\n" * (snippet.line - 1) + snippet.code
    exec(compile(source, str(snippet.path), "exec"), {"__name__": "__main__"})


def check_rust(examples: list[Snippet]) -> None:
    """Compile all Rust fences in one temporary application, then run each."""
    if not examples:
        raise ValueError("No Rust documentation examples found")
    target = ROOT / "target"
    target.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="documentation-", dir=target) as directory:
        crate = Path(directory)
        manifest = crate / "Cargo.toml"
        manifest.write_text(
            '[package]\nname = "crcc-doc-examples"\nversion = "0.0.0"\nedition = "2024"\n'
            '[dependencies]\ngeo = "0.32.0"\n'
            f'crcc = {{ path = {json.dumps(str(ROOT))}, features = ["rayon"] }}\n'
        )
        shutil.copyfile(ROOT / "Cargo.lock", crate / "Cargo.lock")
        binaries = crate / "src" / "bin"
        binaries.mkdir(parents=True)
        for index, snippet in enumerate(examples):
            (binaries / f"example_{index}.rs").write_text(f"// {snippet.label}\n{snippet.code}\n")
        output = target / "doc-examples"
        subprocess.run(
            ["cargo", "build", "--offline", "--manifest-path", str(manifest), "--target-dir", str(output), "--bins"],
            check=True,
            cwd=ROOT,
        )
        for index, snippet in enumerate(examples):
            print(f"Rust: {snippet.label}", flush=True)
            executable = output / "debug" / f"example_{index}{'.exe' if os.name == 'nt' else ''}"
            subprocess.run([str(executable)], check=True, cwd=ROOT)


class PageLinks(HTMLParser):
    """Collect rendered link destinations and anchors, without third-party parsers."""

    def __init__(self) -> None:
        super().__init__()
        self.links: list[str] = []
        self.ids: set[str] = set()

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attributes = dict(attrs)
        if identifier := attributes.get("id"):
            self.ids.add(identifier)
        if tag == "a" and (href := attributes.get("href")):
            self.links.append(href)


def check_site() -> None:
    """Check rendered site links/anchors after Zensical and Rustdoc assembly."""
    site = ROOT / "site"
    cache: dict[Path, PageLinks] = {}

    def page(path: Path) -> PageLinks:
        if path not in cache:
            parser = PageLinks()
            parser.feed(path.read_text())
            cache[path] = parser
        return cache[path]

    count = 0
    for source in sorted((ROOT / "docs").rglob("*.md")):
        relative = source.relative_to(ROOT / "docs")
        rendered = (
            site / relative.parent / "index.html"
            if relative.name == "index.md"
            else site / relative.with_suffix("") / "index.html"
        )
        assert rendered.is_file(), f"Missing rendered page: {rendered}"
        for href in page(rendered).links:
            url = urlsplit(href)
            if url.scheme or url.netloc:
                if not href.startswith("https://burakssen.com/crcc/"):
                    continue
                destination = site / unquote(url.path.removeprefix("/crcc/"))
            elif url.path.startswith("/"):
                destination = site / unquote(url.path.removeprefix("/crcc/").lstrip("/"))
            else:
                destination = rendered.parent / unquote(url.path) if url.path else rendered
            destination = destination.resolve()
            if destination.is_dir():
                destination /= "index.html"
            assert destination.is_file(), f"Broken rendered link in {relative}: {href}"
            if url.fragment and destination.suffix == ".html":
                assert unquote(url.fragment) in page(destination).ids, f"Broken anchor in {relative}: {href}"
            count += 1
    print(f"Verified {count} rendered local links/anchors across documentation pages.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--site", action="store_true", help="Check rendered links only, after building the combined site"
    )
    if parser.parse_args().site:
        check_site()
        return
    examples = snippets()
    python = [snippet for snippet in examples if snippet.language == "python"]
    rust = [snippet for snippet in examples if snippet.language == "rust"]
    for snippet in python:
        print(f"Python: {snippet.label}", flush=True)
        check_python(snippet)
    check_rust(rust)
    print(f"Verified {len(python)} Python and {len(rust)} Rust documentation examples.")


if __name__ == "__main__":
    main()
