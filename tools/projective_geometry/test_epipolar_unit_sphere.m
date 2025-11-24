source "epipolar_unit_sphere.m"

function X = generateTransform()
  R=rand(3,3);
  [U,S,V]=svd(R);
  if (det(U)<0)
    U(:,1)=-U(:,1);
  endif;
  X=[U, rand(3,1);
     0, 0, 0, 1];
endfunction;

function testTriangulatePoints(X,n_points)
  #generate a random set of points
  P=rand(3,n_points)
  
  #project into image
  P1_img=projectPoints(eye(4),P);
  P2_img=projectPoints(X,P);
  [n_points, P_out, errors]=triangulatePoints(X,P1_img, P2_img)
endfunction;


function testEssential(X,n_points)
  #generate a random set of points
  P=rand(3,n_points);

  #project into image
  P1_img=projectPoints(eye(4),P);
  P2_img=projectPoints(X,P);
  
  #get ground truth
  E_gt=transform2essential(X);
  
  #estimate fundamental (w/o outliers)
  E=estimateEssential8pt(P1_img,P2_img)
  disp("essential ratio (up to a scale, should be 1)");
  E/=norm(E);
  E_gt./E
endfunction;

function testTransform(X,n_points, noise_sigma=0)
  #generate a random set of points
  P=rand(3,n_points)*100;

  #project into image
  P1_img=projectPoints(eye(4),P);
  P2_img=projectPoints(X,P);
  #get ground truth
  F=transform2essential(X);

  P1_img+=normrnd(0, noise_sigma, size(P1_img));
  P2_img+=normrnd(0, noise_sigma, size(P2_img));
  
  X_est=estimateTransform(P1_img,P2_img);
  disp("transform original");
  X

  disp("transform estimated");
  X_est

  disp("t ratio")
  X(1:3,4)./X_est(1:3,4)
endfunction;
