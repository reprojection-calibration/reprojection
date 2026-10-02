#!/usr/bin/env python3

import argparse

import pandas as pd
import plotly.graph_objects as go


parser = argparse.ArgumentParser()
parser.add_argument("csv")
args = parser.parse_args()

df = pd.read_csv(args.csv)

fig = go.Figure()

fig.add_trace(go.Scatter(
    x=df["index"],
    y=df["x"],
    mode="lines",
    name="x",
))

fig.add_trace(go.Scatter(
    x=df["index"],
    y=df["y"],
    mode="lines",
    name="y",
))

fig.add_trace(go.Scatter(
    x=df["index"],
    y=df["z"],
    mode="lines",
    name="z",
))

fig.update_layout(
    xaxis_title="Spline control point index",
    yaxis_title="Bias",
    hovermode="x unified",
)

fig.show()