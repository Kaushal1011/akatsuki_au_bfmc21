from re import template
from time import process_time_ns

from matplotlib.pyplot import connect
from dash import Dash, html, dcc
from dash.dependencies import Output, Input
import dash_bootstrap_components as dbc
import plotly.express as px
import pandas as pd
import csv, socket, pickle
from json import loads
import plotly.graph_objects as go

options = dict(loop=True, autoplay=True, rendererSettings=dict(
    preserveAspectRatio='xMidYMid slice'))


app = Dash(__name__, external_stylesheets=[dbc.themes.VAPOR])

app.layout = html.Div([
    dcc.Store(
        id='stored_df',
        data=[],
        storage_type='memory'
    ), dcc.Interval(
        id='html-div',
        interval=100,  # in milliseconds
        n_intervals=0
    ),
    dbc.Container([
        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('X Co-Ordinate'),
                        html.H2(id='x_coord', children="0"),
                        dcc.Interval(
                            id='card-x',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Y Co-Oridnate'),
                        html.H2(id='y_coord', children="0"),
                        dcc.Interval(
                            id='card-y',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('TX Co-Oridnate'),
                        html.H2(id='tx_coord', children="0"),
                        dcc.Interval(
                            id='card-tx',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('TY Co-Oridnate'),
                        html.H2(id='ty_coord', children="0"),
                        dcc.Interval(
                            id='card-ty',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Target Index'),
                        html.H2(id='t_index', children="0"),
                        dcc.Interval(
                            id='card-t-index',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
        ], className='mb-2'),
        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Yaw'),
                        html.H2(id='yaw', children="0"),
                        dcc.Interval(
                            id='card-yaw',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Speed'),
                        html.H2(id='speed', children="0"),
                        dcc.Interval(
                            id='card-speed',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Steering Angle'),
                        html.H2(id='steer_angle', children="0"),
                        dcc.Interval(
                            id='card-steer',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('LK Angle'),
                        html.H2(id='lk_angle', children="0"),
                        dcc.Interval(
                            id='card-lk-angle',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('CS Angle'),
                        html.H2(id='cs_angle', children="0"),
                        dcc.Interval(
                            id='card-cs-angle',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ])
        ], className='mb-3'),
        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Current Behaviour'),
                        html.H2(id='behav', children="Lane Keep"),
                        dcc.Interval(
                            id='card-behav',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Detection'),
                        html.H2(id='detect', children="No Sign"),
                        dcc.Interval(
                            id='card-detect',
                            interval=1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Front Sensor'),
                        html.H2(id='fsu', children="0"),
                        dcc.Interval(
                            id='card-fsu',
                            interval=1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Rear Sensor'),
                        html.H2(id='rsu', children="0"),
                        dcc.Interval(
                            id='card-rsu',
                            interval=1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ])
        ], className='mb-4'),
        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Intersection Detected'),
                        html.H2(id='id-detect', children="False"),
                        dcc.Interval(
                            id='card-id-detect',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Car Detection'),
                        html.H2(id='car-detect', children="False"),
                        dcc.Interval(
                            id='card-car-detect',
                            interval=1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Pedestrian Detected'),
                        html.H2(id='pedo-detect', children="False"),
                        dcc.Interval(
                            id='card-pedo-detec',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        html.H6('Roadblock Detected'),
                        html.H2(id='roadblockdo-detect', children="False"),
                        dcc.Interval(
                            id='card-roadblock-detec',
                            interval=0.1*1000,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ])
        ], className='mb-4'),
        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        dcc.Graph(id='position'), dcc.Interval(
                            id='interval-component-1',
                            interval=1000,  # in milliseconds
                            n_intervals=0
                        )
                    ])
                ]),
            ]),
        ], className='mb-2'),
        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        dcc.Graph(id='speed_curve'), dcc.Interval(
                            id='interval-component-3',
                            interval=1000,  # in milliseconds
                            n_intervals=0
                        ),
                    ])
                ]),
            ]),
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        dcc.Graph(id='yaw_polar'), dcc.Interval(
                            id='interval-component-2',
                            interval=1000,  # in milliseconds
                            n_intervals=0
                        ),
                    ])
                ]),
            ]),
        ], className='mb-2'),
    ], fluid=True)
])


@app.callback(
    Output('stored_df', 'data'),
    Input('html-div', 'n_intervals')
)
def store_data(n_intervals):
    data, addr = sock.recvfrom(4096)
    data = data.decode('utf-8')    
    dictData=loads(data)
    df.append(dictData)
    df_last_20 = df.tail(20)
    print(df_last_20)
    return df_last_20.to_dict('records')


@app.callback(
    Output('x_coord', 'children'),
    Input('stored_df', 'data'),
    Input('card-x', 'n_intervals')
)
def update_card_x(data, n_intervals):
    # top = pd.read_csv("tele.csv", nrows=1)
    # headers = top.columns.values
    # with open("tele.csv", "r") as f, open("tele2.csv", "w+") as g:
    #     last_line = f.readlines()[-1].strip().split(",")
    #     c = csv.writer(g)
    #     c.writerow(headers)
    #     c.writerow(last_line)
    # df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[0]


@app.callback(
    Output('y_coord', 'children'),
    Input('stored_df', 'data'),
    Input('card-y', 'n_intervals')
)
def update_card_y(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[1]

@app.callback(
    Output('tx_coord', 'children'),
    Input('stored_df', 'data'),
    Input('card-tx', 'n_intervals')
)
def update_card_tx(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[1]

@app.callback(
    Output('ty_coord', 'children'),
    Input('stored_df', 'data'),
    Input('card-ty', 'n_intervals')
)
def update_card_ty(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[1]


@app.callback(
    Output('yaw', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_yaw(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[2]


@app.callback(
    Output('speed', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_speed(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[3]


@app.callback(
    Output('steer_angle', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_steer(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[4]

@app.callback(
    Output('behav', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_behav(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[5]


@app.callback(
    Output('detect', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_detect(data, n_intervals):
    # df_tele = pd.read_csv("tele2.csv")
    # # df_tele = pd.DataFrame(data)
    # df_tele_list = df_tele.iloc[-1].tolist()
    dff = pd.DataFrame.from_records(data)
    df_tele_list = dff.iloc[-1].tolist()
    return df_tele_list[6]

# Position ***********************************************************


@app.callback(
    Output('position', 'figure'),
    Input('interval-component-1', 'n_intervals')
)
def update_scatter(n_interval):
    dff = pd.read_csv("tele.csv")
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=dff['X-Coord'], y=dff['Y-Coord'],
                    mode='lines+markers',
                    name='lines+markers'))
    fig.add_trace(go.Scatter(x=dff['TX-Coord'], y=dff['TY-Coord'],
                    mode='lines+markers',
                    name='lines+markers'))
    # fig_line = px.scatter(dff, x='X-Coord', y='Y-Coord', template='plotly_dark',
    #                       title="Position")
    fig.update_layout(template="plotly_dark",margin=dict(l=20, r=20, t=30, b=20))

    return fig


# Polar Yaw Plot ************************************************************
@app.callback(
    Output('yaw_polar', 'figure'),
    Input('stored_df', 'data'),
    Input('interval-component-2', 'n_intervals')
)
def update_polar(data, n_intervals):
    # top = pd.read_csv("tele.csv")
    # df_last_20 = top.tail(20)
    # df_last_20.to_csv("tele3.csv", mode="w+")
    # dff = pd.read_csv("tele3.csv")
    # dff = pd.DataFrame(data)
    dff = pd.DataFrame(data)
    # print(dff)
    fig_polar = px.scatter_polar(dff, r=dff.index, theta="Yaw", template='plotly_dark')
    fig_polar.update_layout(margin=dict(l=20, r=20, t=30, b=20))

    return fig_polar


# Speed Curve ************************************************************
@app.callback(
    Output('speed_curve', 'figure'),
    Input('stored_df', 'data'),
    Input('interval-component-3', 'n_intervals')
)
def update_speed_curve(data, n_intervals):
    # dff = pd.read_csv("tele3.csv")
    # # dff = pd.DataFrame(data)
    dff = pd.DataFrame.from_records(data)
    fig_line = px.line(dff, x=dff.index, y="Speed", template='plotly_dark',
                       title="Speed", markers=True)
    fig_line.update_layout(margin=dict(l=20, r=20, t=30, b=20))

    return fig_line

@app.callback(Output('stored_df', 'clear_data'),
            [Input('html-div', 'n_intervals')])
def clear_dcc_store(n_intervals):
        return True


if __name__ == '__main__':
    
    host = "0.0.0.0"
    port = 12345
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host,port))
    print("Socket Binded")
    df = pd.DataFrame(columns=['X-Coord', 'Y-Coord', 'Yaw', 'Speed', 'Steering_Angle', 'Current_Behaviour', 'Detection'])
    app.run_server(debug=True, port=8004)
    

