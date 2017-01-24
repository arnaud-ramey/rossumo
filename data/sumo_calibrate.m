% https://www.gnu.org/software/octave/doc/interpreter/Linear-Least-Squares.html
% https://stats.stackexchange.com/questions/32504/simple-multivariate-regression-with-octave
% http://sachinashanbhag.blogspot.fr/2012/07/linear-least-squares-in-gnu-octave-part.html
% https://fr.mathworks.com/help/stats/regress.html

M=[25, 0, 0.435, 0
25, -25, 0.4244542560601, 3.1557937253539
25, 50, 0.4703613581911, 7.0203187789716
25, 75, 0.2578206736989, 9.5489138406985
25, 100, 0.1410630938936, 11.7552578244707
50, 0, 1.038, 0
50, 25, 0.9182633181664, 3.1447373909808
50, 50, 0.440203002872, 6.333856156431
50, 75, 0.1849231796093, 8.40559907315
50, 100, 0.0589575028824, 8.4225004117689
75, 0, 1.602587637926, 0
100, 0, 2.066, 0];

lin=M(:,1);
ang=M(:,2);
  v=M(:,3);
  w=M(:,4);

% linear regression (v, w, v*w) -> lin
X=[ones(size(v)), v, w, v .* w];
[bLin, sigma, r]=ols(lin, X);
bLin
%~ bLin =  0.50949 45.38459 -0.81221 3.78223
normR=norm(r)
[1 8 8 8*8]*bLin       % simple prediction
linEst=X*bLin          % check model with initial data

scatter3(v, w, lin, 'filled');
hold on
vfit = min(v):.1:max(v);
wfit = min(w):.1:max(w);
[vFIT,wFIT] = meshgrid(vfit,wfit);
YFIT = bLin(1) + bLin(2)*vFIT + bLin(3)*wFIT + bLin(4)*vFIT.*wFIT;
mesh(vFIT,wFIT,YFIT);
hidden('off')
title('linear regression (v, w, v*w) -> lin')
xlabel('v'); ylabel('w'); zlabel('lin')

% linear regression (v, w, v*w) -> ang
X=[ones(size(v)), v, w, v .* w];
[bAng, sigma, r]=ols(ang, X);
bAng
%~ bAng =  0.98163 -1.82084 11.69517 0.36455
normR=norm(r)
[1 8 8 8*8]*bAng       % simple prediction
angEst=X*bAng          % check model with initial data

figure
scatter3(v, w, ang, 'filled');
hold on
vfit = min(v):.1:max(v);
wfit = min(w):.1:max(w);
[vFIT,wFIT] = meshgrid(vfit,wfit);
YFIT = bAng(1) + bAng(2)*vFIT + bAng(3)*wFIT + bAng(4)*vFIT.*wFIT;
mesh(vFIT,wFIT,YFIT);
hidden('off')
title('linear regression (v, w, v*w) -> ang')
xlabel('v'); ylabel('w'); zlabel('ang')
pause
