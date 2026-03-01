from dataclasses import dataclass
from typing import Literal

import plotly.graph_objects as go
from plotly.subplots import make_subplots


@dataclass
class AxisConfig:
    title: str
    unit: str

    @property
    def full_title(self):
        if self.title and self.unit:
            return f"{self.title} [{self.unit}]"
        else:
            return self.title or self.unit or None


@dataclass
class SubplotConfig:
    title: str
    x_axis: AxisConfig
    y_axis: AxisConfig


# NOTE(Jack): This configuration and the code below is only meant to construct one dimensional figure sets. Either in an
# Nx1 or 1XN configuration.
@dataclass
class FigureConfig:
    title: str
    subplots: tuple[SubplotConfig, ...]
    direction: Literal["rows", "cols"]
    shared_xaxes: bool

    def __post_init__(self):
        if self.direction == "cols" and self.shared_xaxes:
            raise ValueError(
                "Only graphs that physically overlay one another can have a shared x-axis"
            )


def build_figure_layout(config):
    n_subplots = len(config.subplots)
    is_rows = config.direction == "rows"
    n_rows = n_subplots if is_rows else 1
    n_cols = 1 if is_rows else n_subplots

    fig = make_subplots(rows=n_rows, cols=n_cols, shared_xaxes=config.shared_xaxes)

    # NOTE(Jack): We need to add a blank trace otherwise only one of the plots will display.
    for i in range(len(config.subplots)):
        i_row = i + 1 if is_rows else 1
        i_col = 1 if is_rows else i + 1

        fig.add_trace(go.Scatter(x=[], y=[]), row=i_row, col=i_col)

    return fig
