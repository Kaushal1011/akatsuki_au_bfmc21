from dash import Dash, html, dcc
from dash.dependencies import Output, Input
import dash_bootstrap_components as dbc
import plotly.express as px
import pandas as pd
import csv

options = dict(loop=True, autoplay=True, rendererSettings=dict(
    preserveAspectRatio='xMidYMid slice'))

# Bootstrap themes by Ann: https://hellodash.pythonanywhere.com/theme_explorer
app = Dash(__name__, external_stylesheets=[dbc.themes.VAPOR])

app.layout = html.Div([
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
        ], className='mb-2'),
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
                            interval=100,  # in milliseconds
                            n_intervals=0
                        )
                    ], style={'textAlign': 'center'})
                ]),
            ])
        ], className='mb-2'),
        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardBody([
                        dcc.Graph(id='position'), dcc.Interval(
                            id='interval-component-1',
                            interval=100,  # in milliseconds
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
                            interval=100,  # in milliseconds
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
                            interval=100,  # in milliseconds
                            n_intervals=0
                        ),
                    ])
                ]),
            ]),
        ], className='mb-2'),
    ], fluid=True), dcc.Store(
        id='stored_df',
        storage_type='memory'
    ), dcc.Interval(
        id='html-div',
        interval=100,  # in milliseconds
        n_intervals=0
    )
])


@app.callback(
    Output('stored_df', 'data'),
    Input('html-div', 'n_intervals')
)
def store_data(n_intervals):
    df = pd.read_csv('tele.csv')
    df_last_20 = df.tail(20)
    # print(df_last_20.to_dict('records'))
    return df_last_20.to_dict('records')


@app.callback(
    Output('x_coord', 'children'),
    Input('stored_df', 'data'),
    Input('card-x', 'n_intervals')
)
def update_card_x(n_intervals, data):
    top = pd.read_csv("tele.csv", nrows=1)
    headers = top.columns.values
    with open("tele.csv", "r") as f, open("tele2.csv", "w+") as g:
        last_line = f.readlines()[-1].strip().split(",")
        c = csv.writer(g)
        c.writerow(headers)
        c.writerow(last_line)
    df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    df_tele_list = df_tele.iloc[-1].tolist()
    return df_tele_list[1]


@app.callback(
    Output('y_coord', 'children'),
    Input('stored_df', 'data'),
    Input('card-y', 'n_intervals')
)
def update_card_y(n_intervals, data):
    df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    df_tele_list = df_tele.iloc[-1].tolist()
    return df_tele_list[2]


@app.callback(
    Output('yaw', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_yaw(n_intervals, data):
    df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    df_tele_list = df_tele.iloc[-1].tolist()
    return df_tele_list[3]


@app.callback(
    Output('speed', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_yaw(n_intervals, data):
    df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    df_tele_list = df_tele.iloc[-1].tolist()
    return df_tele_list[4]


@app.callback(
    Output('steer_angle', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_yaw(n_intervals, data):
    df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    df_tele_list = df_tele.iloc[-1].tolist()
    return df_tele_list[5]

@app.callback(
    Output('behav', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_yaw(n_intervals, data):
    df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    df_tele_list = df_tele.iloc[-1].tolist()
    return df_tele_list[6]


@app.callback(
    Output('detect', 'children'),
    Input('stored_df', 'data'),
    Input('card-yaw', 'n_intervals')
)
def update_card_yaw(n_intervals, data):
    df_tele = pd.read_csv("tele2.csv")
    # df_tele = pd.DataFrame(data)
    df_tele_list = df_tele.iloc[-1].tolist()
    return df_tele_list[7]

# Position ***********************************************************


@app.callback(
    Output('position', 'figure'),
    Input('interval-component-1', 'n_intervals')
)
def update_scatter(n_interval):
    dff = pd.read_csv("tele.csv")
    fig_line = px.scatter(dff, x='X-Coord', y='Y-Coord', template='plotly_dark',
                          title="Position")
    fig_line.update_layout(margin=dict(l=20, r=20, t=30, b=20))

    return fig_line


# Polar Yaw Plot ************************************************************
@app.callback(
    Output('yaw_polar', 'figure'),
    Input('stored_df', 'data'),
    Input('interval-component-2', 'n_intervals')
)
def update_polar(n_interval, data):
    top = pd.read_csv("tele.csv")
    df_last_20 = top.tail(20)
    df_last_20.to_csv("tele3.csv", mode="w+")
    dff = pd.read_csv("tele3.csv")
    # print(type(data))
    # dff = pd.DataFrame(data)
    fig_polar = px.scatter_polar(dff, r=dff.index, theta="Yaw", template='plotly_dark')
    fig_polar.update_layout(margin=dict(l=20, r=20, t=30, b=20))

    return fig_polar


# Speed Curve ************************************************************
@app.callback(
    Output('speed_curve', 'figure'),
    Input('stored_df', 'data'),
    Input('interval-component-3', 'n_intervals')
)
def update_speed_curve(n_interval, data):
    dff = pd.read_csv("tele3.csv")

    # dff = pd.DataFrame(data)
    fig_line = px.line(dff, x=dff.index, y="Speed", template='plotly_dark', line=dict(color="#ffe476"),
                       title="Speed", markers=True)
    fig_line.update_layout(margin=dict(l=20, r=20, t=30, b=20))

    return fig_line

@app.callback(Output('stored_df', 'clear_data'),
            [Input('html-div', 'n_intervals')])
def clear_click(n_intervals):
        return True


if __name__ == '__main__':
    app.run_server(debug=True, port=8004)
