from dash import Dash, html, dcc
from dash.dependencies import Output, Input
import dash_bootstrap_components as dbc
import plotly.express as px
import pandas as pd
from json import loads
import plotly.graph_objects as go

# from socket import *
import socket

host = "0.0.0.0"
port = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((host, port))
print("Socket Binded")


options = dict(
    loop=True,
    autoplay=True,
    rendererSettings=dict(preserveAspectRatio="xMidYMid slice"),
)


app = Dash(__name__, external_stylesheets=[dbc.themes.VAPOR])

app.layout = html.Div(
    [
        dcc.Store(id="stored_df", data=[], storage_type="memory", clear_data=True),
        dcc.Interval(
            id="store_interval", interval=10, n_intervals=0  # in milliseconds
        ),
        dcc.Interval(id="mem_clear", interval=100, n_intervals=0),  # in milliseconds
        dbc.Container(
            [
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("X Co-Ordinate"),
                                                html.H2(id="x_coord", children="0"),
                                                dcc.Interval(
                                                    id="cards",
                                                    interval=10,  # in milliseconds
                                                    n_intervals=0,
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Y Co-Oridnate"),
                                                html.H2(id="y_coord", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("RX Co-Oridnate"),
                                                html.H2(id="rx_coord", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("RY Co-Oridnate"),
                                                html.H2(id="ry_coord", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("TX Co-Oridnate"),
                                                html.H2(id="tx_coord", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("TY Co-Oridnate"),
                                                html.H2(id="ty_coord", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Target Index"),
                                                html.H2(id="t_index", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                    ],
                    className="mb-2",
                ),
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Yaw"),
                                                html.H2(id="yaw", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Pitch"),
                                                html.H2(id="pitch", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Roll"),
                                                html.H2(id="roll", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Speed"),
                                                html.H2(id="speed", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Steering Angle"),
                                                html.H2(id="steer_angle", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("LK Angle"),
                                                html.H2(id="lk_angle", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("CS Angle"),
                                                html.H2(id="cs_angle", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                    ],
                    className="mb-3",
                ),
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Current PType"),
                                                html.H2(
                                                    id="ptype",
                                                    children="Lane Keep / CS",
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Current Target"),
                                                html.H2(id="target", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Current Behaviours"),
                                                html.H2(
                                                    id="behav", children="Lane Keep"
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Detection"),
                                                html.H2(
                                                    id="detect", children="No Sign"
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Front Sensor"),
                                                html.H2(id="fsu", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Rear Sensor"),
                                                html.H2(id="rsu", children="0"),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                    ],
                    className="mb-4",
                ),
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Intersection Detected"),
                                                html.H2(
                                                    id="id-detect", children="False"
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Car Detected"),
                                                html.H2(
                                                    id="car-detect", children="False"
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Pedestrian Detected"),
                                                html.H2(
                                                    id="pedo-detect", children="False"
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                html.H6("Roadblock Detected"),
                                                html.H2(
                                                    id="roadblock-detect",
                                                    children="False",
                                                ),
                                            ],
                                            style={"textAlign": "center"},
                                        )
                                    ]
                                ),
                            ]
                        ),
                    ],
                    className="mb-4",
                ),
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                dbc.Card(
                                    [
                                        dbc.CardBody(
                                            [
                                                dcc.Graph(id="position"),
                                                dcc.Interval(
                                                    id="graph-update",
                                                    interval=10,  # in milliseconds
                                                    n_intervals=0,
                                                ),
                                            ]
                                        )
                                    ]
                                ),
                            ]
                        ),
                    ],
                    className="mb-2",
                ),
                dbc.Row(
                    [
                        dbc.Col(
                            [
                                dbc.Card([dbc.CardBody([dcc.Graph(id="speed_curve")])]),
                            ],
                            width=6,
                        ),
                        dbc.Col(
                            [
                                dbc.Card([dbc.CardBody([dcc.Graph(id="yaw_polar")])]),
                            ],
                            width=6,
                        ),
                    ],
                    className="mb-2",
                ),
            ],
            fluid=True,
        ),
    ]
)


# @app.callback(
#     Output('stored_df', 'data'),
#     Input('html-div', 'n_intervals')
# )
# def store_data(n_intervals):
#     # data, addr = sock.recvfrom(4096)
#     # data = data.decode('utf-8')
#     # dictData=loads(data)
#     # print(dictData)
#     # df.append(dictData)
#     # print(df)
#     # df_last_20 = df.tail(20)
#     # print(df_last_20)
#     return dictData


@app.callback(
    Output("x_coord", "children"),
    Output("y_coord", "children"),
    Output("rx_coord", "children"),
    Output("ry_coord", "children"),
    Output("tx_coord", "children"),
    Output("ty_coord", "children"),
    Output("t_index", "children"),
    Output("yaw", "children"),
    Output("pitch", "children"),
    Output("roll", "children"),
    Output("speed", "children"),
    Output("steer_angle", "children"),
    Output("lk_angle", "children"),
    Output("cs_angle", "children"),
    Output("ptype", "children"),
    Output("target", "children"),
    Output("behav", "children"),
    Output("detect", "children"),
    Output("fsu", "children"),
    Output("rsu", "children"),
    Output("id-detect", "children"),
    Output("car-detect", "children"),
    Output("pedo-detect", "children"),
    Output("roadblock-detect", "children"),
    Input("cards", "n_intervals"),
)
def update_cards(n_intervals):
    data, _ = sock.recvfrom(4096)
    data = data.decode("utf-8")
    dictData = loads(data)
    # x =
    # y =
    # yaw =
    # speed = dictData["v"]
    # pitch =
    # roll = dictData["roll"]
    # rx =

    # ty = dictData["target_y"]
    # tidx =
    # steer = dictData["steering_angle"]
    # lk_angle = dictData["lk_angle"]
    # cs_angle = dictData["cs_angle"]
    # fsu = dictData["front_distance"]
    # rsu = dictData["side_distance"]
    # ptype = dictData["current_ptype"]
    # curr_target = dictData["current_target"]
    # id_detect = dictData["detected_intersection"]
    # car_detect = dictData["car_detected"]
    # behav = dictData["active_behaviours"]
    # roadblock = dictData["roadblock"]
    # pedo = dictData["pedestrian"]
    stop = dictData["stop"]
    crosswalk = dictData["crosswalk"]
    highway_entry = dictData["highway_entry"]
    highway_exit = dictData["highway_exit"]
    priority = dictData["priority"]
    one_way = dictData["onewayroad"]
    parking = dictData["parking"]
    no_entry = dictData["no_entry"]
    roundabout = dictData["roundabout"]
    sign = []
    if stop:
        sign.append("stop")
    elif crosswalk:
        sign.append("crosswalk")
    elif highway_entry:
        sign.append("highway entry")
    elif highway_exit:
        sign.append("highway exit")
    elif priority:
        sign.append("priority")
    elif one_way:
        sign.append("one way")
    elif parking:
        sign.append("parking")
    elif no_entry:
        sign.append("no entry")
    elif roundabout:
        sign.append("roundabout")
    else:
        sign = ["no sign"]

    # print(dictData)

    return (
        dictData["x"],
        dictData["y"],
        dictData["rear_x"],
        dictData["rear_y"],
        dictData["target_x"],
        dictData["target_y"],
        dictData["target_idx"],
        dictData["yaw"],
        dictData["pitch"],
        dictData["roll"],
        dictData["v"],
        dictData["steering_angle"],
        dictData["lk_angle"],
        dictData["cs_angle"],
        dictData["current_ptype"],
        dictData["current_target"],
        dictData["active_behaviours"],
        sign,
        dictData["front_distance"],
        dictData["side_distance"],
        dictData["detected_intersection"],
        dictData["car_detected"],
        dictData["pedestrian"],
        dictData["roadblock"],
        # ry,
        # tx,
        # ty,
        # tidx,
        # yaw,
        # pitch,
        # roll,
        # speed,
        # steer,
        # lk_angle,
        # cs_angle,
        # ptype,
        # curr_target,
        # behav,
        # sign,
        # fsu,
        # rsu,
        # str(id_detect),
        # str(car_detect),
        # str(pedo),
        # str(roadblock),
    )


# =====Graphs===== #


@app.callback(
    Output("position", "figure"),
    Output("yaw_polar", "figure"),
    Output("speed_curve", "figure"),
    Input("graph-update", "n_intervals"),
)
def update_scatter(n_interval):
    data, _ = sock.recvfrom(4096)
    data = data.decode("utf-8")
    data = loads(data)
    # print(data)
    in_list = [
        data["x"],
        data["y"],
        data["yaw"],
        data["v"],
        data["rear_x"],
        data["rear_y"],
        data["target_x"],
        data["target_y"],
    ]
    dff.loc[len(dff)] = in_list
    print(dff)
    # =====Position=====#

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            x=dff["x"], y=dff["y"], mode="lines+markers", name="CUrrent Position"
        )
    )
    fig.add_trace(go.Scatter(x=dff["target_x"], y=dff["target_y"], name="Target Index"))

    fig.update_layout(template="plotly_dark", margin=dict(l=20, r=20, t=30, b=20))

    # =====Yaw=====#

    fig_polar = px.scatter_polar(dff, r=dff.index, theta="yaw", template="plotly_dark")
    fig_polar.update_layout(margin=dict(l=20, r=20, t=30, b=20))

    # =====Speed=====#

    fig_speed = go.Figure(
        go.Indicator(
            mode="gauge+number",
            value=data["v"],
            domain={"x": [0, 1], "y": [0, 1]},
            title={"text": "Speed", "font": {"size": 24}},
            gauge={
                "axis": {"range": [0, 30], "tickwidth": 1, "tickcolor": "darkblue"},
                "bar": {"color": "darkblue"},
                "bgcolor": "white",
                "borderwidth": 2,
                "bordercolor": "gray",
                "steps": [
                    {"range": [0, 10], "color": "green"},
                    {"range": [10, 20], "color": "yellow"},
                    {"range": [20, 30], "color": "red"},
                ],
                "threshold": {
                    "line": {"color": "black", "width": 4},
                    "thickness": 0.75,
                    "value": 29,
                },
            },
        )
    )

    fig_speed.update_layout(
        template="plotly_dark",
        paper_bgcolor="darkblue",
        font={"color": "darkblue", "family": "Arial"},
    )

    return fig, fig_speed, fig_polar


@app.callback(Output("stored_df", "clear_data"), [Input("mem_clear", "n_intervals")])
def clear_dcc_store(n_intervals):
    return True


if __name__ == "__main__":
    dff = pd.DataFrame(
        columns=["x", "y", "yaw", "v", "rear_x", "rear_y", "target_x", "target_y"]
    )
    app.run_server(debug=True, port=8004)
