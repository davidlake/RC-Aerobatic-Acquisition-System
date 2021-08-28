% q1 = quaternion([0,-90,0],'eulerd','XYZ','point')
% q2 = quaternion([-45,0,0],'eulerd','XYZ','point')

q1 = quaternion([0,-90,0],'eulerd','XYZ','point')
q2 = quaternion([0,0,0],'eulerd','XYZ','point')

q3 = q2*q1 % [90,0,90]

eulerd(q3,'XYZ','point')