from sympy import *
x, y, z = symbols('x,y,z')
init_printing(use_unicode=False, wrap_line=False)


f = x**5+x**4-x**3+x**2+x+1
#f = x**4-2*x+1


# print(f.as_poly())
# print(diff(f, x).subs(x, 0))

def get_velocity_vector(t):
    """
    At time t get direction vector
    """
    velocity_scale = 10
    x_val = 1
    y_val = diff(f, x).subs(x, t-2)
    normalize = (abs(x_val) + abs(y_val)) / 2
    
    return [velocity_scale * (x_val / normalize), velocity_scale * (y_val / normalize), 0]
