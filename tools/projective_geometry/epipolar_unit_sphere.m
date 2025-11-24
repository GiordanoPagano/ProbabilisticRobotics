source "../utilities/geometry_helpers_3d.m"

1;

function P_out=transformPoints(X, P_in)
  R=X(1:3,1:3);
  t=X(1:3,4);
  P_out = R*P_in+repmat(t,1, size(P_in,2));
endfunction;


%projects a set of world points onto the image plane
function P_img=projectPoints(X,P_in)
  P_img=transformPoints(inverse(X),P_in);
  P_img2 = sqrt(sum(P_img.*P_img)); # squares
  P_img  = P_img ./ P_img2;
endfunction;

% triangulates a point, passing through two lines
% one passing through the origin, and having
% direction vector d1
% one passing through a point p2, and having
% direction d2
% p: the triangulated point
% e: the error (-1 if behind some of the cameras)
function [p, e]=triangulatePoint(p2, d1, d2)
  p=zeros(3,1);
  success=false;
  e=-1;
                      
  D=[-d1, d2];         #assemble system matrix to find ascissa 
  s=-(D'*D)\(D'*p2);          #s: ascissa of closest point on p1 and p2
  if (s(1)<0 || s(2)<0)
    return;
  endif;
  p1_triangulated=d1*s(1);   # point on 1st line
  p2_triangulated=d2*s(2)+p2; # point on 2nd line
  e=norm(p1_triangulated-p2_triangulated); #difference between the points
  p=0.5*(p1_triangulated+p2_triangulated);               #midpoint
endfunction;

# triangulates a batch of points in image coords,
# X: is the pose of 2nd camera w.r.t world
function [n_success, P, errors] = triangulatePoints(X,P1_img,P2_img)
  #initialize vars
  n_points=size(P1_img,2);
  P=zeros(3,n_points);
  errors=zeros(1,n_points);
  n_success= 0;
  
  #inverse transform
  R=X(1:3,1:3);
  t=X(1:3,4);
  
  for (i=1:n_points)
    p1_cam=P1_img(:,i);
    p2_cam=R*P2_img(:,i);
    [p, e]=triangulatePoint(t, p1_cam, p2_cam);
    if (e>=0)
      ++n_success;
    endif;
    P(:,i)=p;
    errors(:,i)=e;
  endfor;
endfunction

function  E = transform2essential(X)
  E=X(1:3,1:3)'*skew(X(1:3,4));
endfunction;


function [X1, X2]=essential2transform(E)
  W=[0, -1,  0;
     1,  0,  0;
     0,  0,  1];

  [U,S,V]=svd(E);
  R1=V*W*U';
  if (det(R1)<0) #right handed condition
    [U,S,V]=svd(-E);
    R1=V*W*U';
  endif;
  #1st solution for the rotation
  X1=eye(4);
  X1(1:3,1:3)=R1;
  t_cross=R1*E;
  X1(1:3,4)= [t_cross(3,2)-t_cross(2,3);
              t_cross(1,3)-t_cross(3,1);
              t_cross(2,1)-t_cross(1,2)];

  #2nd solution for the rotation
  R2=V*W'*U';
  X2=eye(4);
  X2(1:3,1:3)=R2;
  t_cross=R2*E;
  X2(1:3,4)= [t_cross(3,2)-t_cross(2,3);
              t_cross(1,3)-t_cross(3,1);
              t_cross(2,1)-t_cross(1,2)];
endfunction;

#estimate essential matrix
function E = estimateEssential8pt(P1_img, P2_img)
  H=zeros(9,9);
  n_points=size(P1_img,2);
  for (i=1:n_points)
    p1_img=P1_img(:,i);
    p2_img=P2_img(:,i);
    A=reshape(p2_img*p1_img',1,9);
    H+=A'*A;
  endfor;
  [V,lambda]=eig(H);
  E=reshape(V(:,1),3,3);
endfunction

function X=estimateTransform(P1_img, P2_img)
  # extract essential
  E=estimateEssential8pt(P1_img,P2_img);

  #extract transforms from essential
  [X1,X2]=essential2transform(E);

  X=X1;
  n_in_front=0;
  #for each transform pick the best
  X_test=X1;
  [n_test, P, errors]=triangulatePoints(X_test,P1_img, P2_img);
  if (n_test>n_in_front)
    X=X_test;
    n_in_front=n_test;
  endif;
  X_test(1:3,4)=-X_test(1:3,4);
  [n_test, P, errors]=triangulatePoints(X_test,P1_img, P2_img);
  if (n_test>n_in_front)
    X=X_test;
    n_in_front=n_test;
  endif;
  X_test=X2;
  [n_test, P, errors]=triangulatePoints(X_test,P1_img, P2_img);
  if (n_test>n_in_front)
    X=X_test;
    n_in_front=n_test;
  endif;
  X_test(1:3,4)=-X_test(1:3,4);
  [n_test, P, errors]=triangulatePoints(X_test,P1_img, P2_img);
  if (n_test>n_in_front)
    X=X_test;
    n_in_front=n_test;
  endif;
endfunction;
