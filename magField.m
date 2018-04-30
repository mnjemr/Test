% Magnetic field model
function b=magField(wot,In,Gta,Beta1,kmg)
    cosepsm=cos(In)*cos(Gta)+sin(In)*sin(Gta)*cos(Beta1);
    nim=atan2(-sin(Gta)*sin(Beta1),sin(In)*cos(Gta)-...
    cos(In)*sin(Gta)*cos(Beta1));
    if sin(nim)==0;
        sinepsm=sin(In)*cos(Gta)-cos(In)*sin(Gta)*cos(Beta1)/cos(nim);
    else
        sinepsm=-sin(Gta)*sin(Beta1)/sin(nim);
    end
    b=kmg*[sinepsm*cos(wot-nim);-cosepsm;2*sinepsm*sin(wot-nim)];
end