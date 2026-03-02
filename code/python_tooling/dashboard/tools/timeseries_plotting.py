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
    n_traces: int


# NOTE(Jack): This configuration and the code below is only meant to construct one dimensional figure sets. Either in an
# Nx1 or 1XN configuration.
@dataclass
class FigureConfig:
    title: str
    subplots: tuple
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

    # TODO(Jack): Do we need to set these here or can we add them later when we iterate over the subplots like we do
    #  for the axis properties?
    subplot_titles = [sp.title for sp in config.subplots]
    fig = make_subplots(
        rows=n_rows,
        cols=n_cols,
        subplot_titles=subplot_titles,
        shared_xaxes=config.shared_xaxes,
    )
    fig.update_layout(
        title_text=config.title,
    )

    for i, subplot_config in enumerate(config.subplots):
        i_row = i + 1 if is_rows else 1
        i_col = 1 if is_rows else i + 1

        # TODO(Jack): Can we remove the x-axis title when the axis is shared? I.e. for timeseries plotting when the
        #  times are the same?
        fig.update_xaxes(
            title_text=subplot_config.x_axis.full_title, row=i_row, col=i_col
        )
        fig.update_yaxes(
            title_text=subplot_config.y_axis.full_title, row=i_row, col=i_col
        )

        for i in range(subplot_config.n_traces):
            # TODO(Jack): Make webgl scatter trace?
            fig.add_trace(go.Scatter(x=[], y=[], mode="markers"), row=i_row, col=i_col)

    return fig
