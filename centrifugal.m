function N = centrifugal(q,qd)

% Calculate the Coriolis and centrifugal forces

s2 = sin(q(2));
s3 = sin(q(3));
s23 = sin(q(2)+q(3));

n11 = -.1742*s2*qd(2)^2-0.04047*s23*qd(2)^2-.02809*s3*qd(3)^2-.04047*s23*qd(3)^2-0.3484*s2*qd(1)*qd(2)-.08094*s23*qd(1)*qd(2);
n12 = -0.05619*s3*qd(1)*qd(3)-.08094*s23*qd(1)*qd(3)-.05619*s3*qd(2)*qd(3)-.08094*s23*qd(2)*qd(3);
n2 = .1742*s2*qd(1)^2+.04047*s23*qd(1)^2-.02809*s3*qd(3)^2-0.0561*s3*qd(1)*qd(3)-0.05619*s3*qd(2)*qd(3);
n3 = 0.02809*s3*qd(1)^2+.02809*s3*qd(2)^2+.04047*s23*qd(1)^2+.05619*s3*qd(1)*qd(2);

N = [n11+n12; n2; n3];

end

