fmu_acc_0 = [-2.42, -1.92, -9.25]';
camera_acc_0 = [-1, -0.36, -10.05]';

fmu_AccAngle(1,1) = atan2(-fmu_acc_0(2), -fmu_acc_0(3)); % roll
fmu_AccAngle(2,1) = atan2(fmu_acc_0(1), sqrt(fmu_acc_0(2)^2 + fmu_acc_0(3)^2));
fmu_AccAngle(3,1) = 0;
R_fmu = funAtt2Rnb( fmu_AccAngle );

camera_AccAngle(1,1) = atan2(-camera_acc_0(2), -camera_acc_0(3)); % roll
camera_AccAngle(2,1) = atan2(camera_acc_0(1), sqrt(camera_acc_0(2)^2 + camera_acc_0(3)^2));
camera_AccAngle(3,1) = 0;
R_camera = funAtt2Rnb( camera_AccAngle );

R_fmu2camera = R_camera*R_fmu';


R_fmu2camera =[ 0.99995128537185         -0.00200603819257      -0.00966450691732944
                 0.0003533437327225          0.98577734928264        -0.168056219133655
                 0.00986417920516978         0.168044617444483         0.985730036326654];