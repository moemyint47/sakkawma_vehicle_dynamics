"""Derivation log.

Every derived number in the tool is produced through a ``Calc`` object so
that the formula (LaTeX), the substituted values and the result are kept
together and can be shown to the user ("show derivation") or exported in
the full calculation report.
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any


def fmt(x: float, sig: int = 6) -> str:
    """Format a number for display inside LaTeX substitutions."""
    if x is None:
        return r"\text{n/a}"
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return str(x)
    if xf != xf:  # NaN
        return r"\text{NaN}"
    if xf == 0:
        return "0"
    a = abs(xf)
    if a >= 1e5 or a < 1e-3:
        m, e = f"{xf:.{sig - 1}e}".split("e")
        return rf"{m}\times10^{{{int(e)}}}"
    s = f"{xf:.{sig}g}"
    if "e" in s:  # fallback for g-format switching to exponent
        s = f"{xf:.{sig}f}".rstrip("0").rstrip(".")
    return s


def paren(x: float, sig: int = 6) -> str:
    """Number wrapped in parentheses when negative (for substitutions)."""
    s = fmt(x, sig)
    return f"({s})" if float(x) < 0 else s


@dataclass
class Step:
    key: str            # unique id, e.g. "front.dFz_geo"
    symbol: str         # LaTeX symbol, e.g. r"\Delta F_{z,g}^{f}"
    label: str          # human-readable name
    formula: str        # symbolic LaTeX right-hand side
    subst: str          # LaTeX right-hand side with numbers substituted
    value: Any          # numeric result (SI or stated unit)
    unit: str           # unit string for display
    section: str        # grouping in the report
    ref: str = ""       # source reference (book equation etc.)
    note: str = ""      # assumptions / remarks

    def to_dict(self) -> dict:
        return asdict(self)


class Calc:
    """Collects derivation steps in evaluation order."""

    def __init__(self) -> None:
        self.steps: list[Step] = []
        self._index: dict[str, Step] = {}
        self.warnings: list[str] = []

    def add(self, key, symbol, label, formula, subst, value, unit, section,
            ref: str = "", note: str = ""):
        st = Step(key, symbol, label, formula, subst, value, unit, section, ref, note)
        if key in self._index:  # overwrite, keep order of first appearance
            i = self.steps.index(self._index[key])
            self.steps[i] = st
        else:
            self.steps.append(st)
        self._index[key] = st
        return value

    def input(self, key, symbol, label, value, unit, section, note=""):
        """Record a user input (shown in report for completeness)."""
        return self.add(key, symbol, label, r"\text{input}", fmt(value), value,
                        unit, section, note=note)

    def warn(self, msg: str) -> None:
        if msg not in self.warnings:
            self.warnings.append(msg)

    def get(self, key):
        return self._index[key].value

    def to_list(self) -> list[dict]:
        return [s.to_dict() for s in self.steps]


class NullCalc(Calc):
    """Drop-in replacement that skips bookkeeping (fast sweeps)."""

    def add(self, key, symbol, label, formula, subst, value, *a, **k):
        return value

    def input(self, key, symbol, label, value, *a, **k):
        return value
