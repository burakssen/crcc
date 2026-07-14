ResultRow = tuple[str, str, str]


def collision_result(name, query, true="hit", false="clear") -> ResultRow:
    try:
        value = query()
        return name, true if value else false, f"collides={value}"
    except Exception as error:
        return name, "unsupported", f"{type(error).__name__}: {error}"


def print_results(title: str, results: tuple[ResultRow, ...]) -> None:
    print(title)
    for name, outcome, detail in results:
        print(f"  {name:<28} {outcome:<19} {detail}")
