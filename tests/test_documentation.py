"""Check executable docs, navigation, and native/public stub contract drift."""

import ast
import importlib
import inspect
import re

import pytest

from tools.check_documentation import ROOT, check_python, snippets


@pytest.mark.parametrize("snippet", [s for s in snippets() if s.language == "python"], ids=lambda s: s.label)
def test_documented_python_example(snippet):
    check_python(snippet)


def test_navigation_and_local_page_links():
    docs = ROOT / "docs"
    config = (ROOT / "zensical.toml").read_text().split("[project.theme]")[0]
    nav = re.findall(r'"([^"\n]+\.md)"', config)
    pages = {str(path.relative_to(docs)) for path in docs.rglob("*.md")}
    assert set(nav) == pages, "Every documentation page must have an intentional navigation entry"
    assert len(nav) == len(set(nav)), "Duplicate navigation entry"
    for path in docs.rglob("*.md"):
        for link in re.findall(r"\]\(([^\s)]+)\)", path.read_text()):
            if "://" in link or link.startswith("#"):
                continue
            destination = link.split("#")[0]
            if destination:
                resolved = (path.parent / destination).resolve()
                assert resolved.is_relative_to(docs), f"Repository link must use an absolute URL: {path}: {link}"
                assert resolved.is_file(), f"Broken page link: {path}: {link}"


def stub_paths():
    return sorted((ROOT / "python" / "crcc").rglob("*.pyi"))


def test_stub_members_and_keyword_signatures_match_runtime():
    """Inspect declared classes/functions, including the native builder boundary."""
    for path in stub_paths():
        parts = path.relative_to(ROOT / "python").with_suffix("").parts
        module_name = ".".join(parts[:-1] if parts[-1] == "__init__" else parts)
        module = importlib.import_module(module_name)
        for node in ast.parse(path.read_text()).body:
            if isinstance(node, ast.ClassDef):
                runtime = getattr(module, node.name)
                functions = [item for item in node.body if isinstance(item, ast.FunctionDef)]
            elif isinstance(node, ast.FunctionDef):
                runtime = module
                functions = [node]
            else:
                continue
            for function in functions:
                member = runtime if function.name == "__init__" else getattr(runtime, function.name)
                decorators = {getattr(decorator, "id", "") for decorator in function.decorator_list}
                if "property" in decorators or function.name in {"NoCollision", "CollidesStatic", "CollidesDynamic"}:
                    continue  # Generated enum constructors/properties have no ordinary function signature.
                assert callable(member)
                try:
                    signature = inspect.signature(member)
                except ValueError:
                    continue  # Some generated special methods omit introspection metadata.
                arguments = [*function.args.posonlyargs, *function.args.args, *function.args.kwonlyargs]
                expected = [argument.arg for argument in arguments if argument.arg not in {"self", "cls"}]
                actual = [parameter for name, parameter in signature.parameters.items() if name not in {"self", "cls"}]
                label = f"{module_name}.{node.name}.{function.name}"
                assert len(expected) == len(actual), f"Argument count drift in {label}"
                for name, parameter in zip(expected, actual, strict=True):
                    # PyO3's operator slots use generated positional-only names,
                    # e.g. __mul__(value, /); those names are not keyword contracts.
                    if parameter.kind != inspect.Parameter.POSITIONAL_ONLY:
                        assert name == parameter.name, f"Keyword signature drift in {label}"


def test_root_reference_contains_every_supported_export():
    import crcc

    reference = (ROOT / "docs" / "reference" / "python.md").read_text()
    for name in crcc.__all__:
        assert re.search(rf"\b{re.escape(name)}\b", reference), f"Missing public reference: {name}"
