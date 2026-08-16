import pytest


def pytest_collection_modifyitems(config: pytest.Config, items: list[pytest.Item]) -> None:
    selected = config.getoption("-m") or ""
    if "live" in selected:
        return
    skip_live = pytest.mark.skip(reason="requires running KSP + kRPC server (run with -m live)")
    for item in items:
        if "live" in item.keywords:
            item.add_marker(skip_live)
