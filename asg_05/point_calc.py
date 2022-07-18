(Px, Py) = (1, 2)
(Cx, Cy) = (7, 5)
a = 2

from math import sqrt, acos, atan2, sin, cos

b = sqrt((Px - Cx)**2 + (Py - Cy)**2)  # hypot() also works here
th = acos(a / b)  # angle theta
d = atan2(Py - Cy, Px - Cx)  # direction angle of point P from C
d1 = d + th  # direction angle of point T1 from C
d2 = d - th  # direction angle of point T2 from C

T1x = Cx + a * cos(d1)
T1y = Cy + a * sin(d1)
T2x = Cx + a * cos(d2)
T2y = Cy + a * sin(d2)

print(T1x, T1y)
print(T2x, T2y)