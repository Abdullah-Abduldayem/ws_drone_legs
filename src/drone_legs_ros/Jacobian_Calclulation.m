pkg load symbolic
syms L1 L2 a11 a12 a21 a22 a31 a32 a41 a42 r p Delta_42 Delta_31 d
% r = roll
% p = pitch
% d = diagonal distance of top plate mounting points
% a21 = Leg 2, angle 1 (ie. thigh lift of front-left leg)


% leg2    leg1 (pitch, ccw)
%   \      /       
%    \____/          /y
%    |   |          /
%    |___|          \
%    /   \           \x
%   /     \
% leg3   leg4 (roll, ccw)
f = @(x1, x2, trig) L1*trig(x1)+L2*trig(x1+x2);


leg1 = [
  f(a11, a12, @cos)*sin(r)*sin(p) + f(a11, a12, @sin)*cos(r)*sin(p),
  f(a11, a12, @cos)*cos(r)        - f(a11, a12, @sin)*sin(r),
  f(a11, a12, @cos)*sin(r)*cos(p) + f(a11, a12, @sin)*cos(r)*cos(p)
  ];


leg2 = [
  -f(a21, a22, @cos)*cos(p) + f(a21, a22, @sin)*cos(r)*sin(p),
   0                        - f(a21, a22, @sin)*sin(r),
   f(a21, a22, @cos)*sin(p) + f(a21, a22, @sin)*cos(r)*cos(p)
  ];
  
leg3 = [
  -f(a31, a32, @cos)*sin(r)*sin(p) + f(a31, a32, @sin)*cos(r)*sin(p),
  -f(a31, a32, @cos)*cos(r)        - f(a31, a32, @sin)*sin(r),
  -f(a31, a32, @cos)*sin(r)*cos(p) + f(a31, a32, @sin)*cos(r)*cos(p)
  ];

leg4 = [
   f(a41, a42, @cos)*cos(p) + f(a41, a42, @sin)*cos(r)*sin(p),
   0                        - f(a41, a42, @sin)*sin(r),
  -f(a41, a42, @cos)*sin(p) + f(a41, a42, @sin)*cos(r)*cos(p)
  ];


r_local = [
  f(a11, a12, @sin),
  f(a21, a22, @sin),
  f(a31, a32, @sin),
  f(a41, a42, @sin)
  ]
  
z_local = [
  f(a11, a12, @cos),
  f(a21, a22, @cos),
  f(a31, a32, @cos),
  f(a41, a42, @cos)
]
  
x = leg1(1)+leg2(1)+leg3(1)+leg4(1);
y = leg1(2)+leg2(2)+leg3(2)+leg4(2);
z = (leg1(3)+leg2(3)+leg3(3)+leg4(3))/4;
roll = acos((Delta_31-leg3(3)+leg1(3))/d);
pitch = acos((Delta_42-leg4(3)+leg2(3))/d);
d1 = (r_local(1)-r_local(3))**2 + (z_local(1)-z_local(3));
d2 = (r_local(2)-r_local(4))**2 + (z_local(2)-z_local(4));

#roll = acos((-z_local(3)+z_local(1))/d)
#pitch = acos((-z_local(4)+z_local(2))/d)

#diff(roll, a11)