from pydrake.common.value import Value
from pydrake.math import RigidTransform, RotationMatrix

a =lambda: Value(RigidTransform()) 

print(a)