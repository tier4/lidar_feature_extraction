from pathlib import Path
from sympy import diff, symbols, Matrix, simplify
from sympy.algebras.quaternion import Quaternion
from sympy.utilities.codegen import codegen
from sympy.printing import ccode

w,x,y,z = symbols("w,x,y,z")
a,b,c = symbols("a,b,c")

def jacobian(w, x, y, z, a, b, c):
    p = Matrix(3, 1, [a, b, c])
    q = Matrix(4, 1, [w, x, y, z])
    R = Quaternion(w, x, y, z).to_rotation_matrix()
    return (R * p).jacobian(q)

def generate_drp_dq():
    JQ = jacobian(w, x, y, z, a, b, c)
    codegen(("drp_dq", JQ), "C99", "drp_dq", header=True, empty=True, to_files=True)

def generatejwxyz():
    R = Quaternion(w, x, y, z).to_rotation_matrix()
    JW = simplify(diff(R, w))
    JX = simplify(diff(R, x))
    JY = simplify(diff(R, y))
    JZ = simplify(diff(R, z))
    codegen(("jw", JW), "C99", "jacobian/jw", header=True, empty=True, to_files=True)
    codegen(("jx", JX), "C99", "jacobian/jx", header=True, empty=True, to_files=True)
    codegen(("jy", JY), "C99", "jacobian/jy", header=True, empty=True, to_files=True)
    codegen(("jz", JZ), "C99", "jacobian/jz", header=True, empty=True, to_files=True)

# generatejwxyz()

generate_drp_dq()

h = Path("drp_dq.h")
h.rename("jacobian/include/jacobian/" + h.name)

c = Path("drp_dq.c")
c.rename("jacobian/src/" + c.name)
