def dist_x(u_x, a_x, dt):
    s_x = u_x*dt + 0.5*a_x*dt**2
    return s_x

def dist_y(u_y, a_y, dt):
    s_y = u_y*dt + 0.5*a_y*dt**2
    return s_y

def vel_x(u_x, a_x, dt):
    v_x = u_x + a_x*dt
    return v_x

def vel_y(u_y, a_y, dt):
    v_y = u_y + a_y*dt
    return v_y


u_init_x = 0
u_init_y = 0
a_x = 2
a_y = 2
dt = 0.2

if __name__ == "__main__":
    v_x1 = vel_x(u_init_x, a_x, dt)
    v_y1 = vel_y(u_init_y, a_y, dt)
    s_x1 = dist_x(v_x1, a_x, dt)
    s_y1 = dist_x(v_y1, a_y, dt)
    v_x = vel_x(v_x1, a_x, dt)
    v_y = vel_y(v_y1, a_y, dt)
    s_x = dist_x(v_x, a_x, dt)
    s_y = dist_x(v_x, a_y, dt)

    while True:
        v_x = vel_x(v_x, a_x, dt)
        v_y = vel_y(v_x, a_y, dt)
        s_x1 = dist_x(v_y, a_x, dt)
        s_y1 = dist_x(v_y, a_y, dt)