function out = sgn(vec,gain,bias)
%SGN 此处显示有关此函数的摘要
%   此处显示详细说明
out = gain*(vec)/(norm(vec,2)+bias);
end

