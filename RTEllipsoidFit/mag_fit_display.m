% display

function mag_fit_display(x, y, z, center, radii, v, xCorr, yCorr, zCorr)

displayRadius = 100;

figure(1);
hold on;

grid;
xlabel('x');
ylabel('y');
zlabel('z');
axis ([-displayRadius, displayRadius, -displayRadius, displayRadius, -displayRadius, displayRadius], "square");
title('Uncorrected samples (red) and corrected (blue) values');
plot3( x, y, z, ['.' 'r'] );
plot3( xCorr, yCorr, zCorr, ['.' 'b'] );

% display immediately

drawnow();

end
