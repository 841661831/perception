cd /home/$USER/WJ-ISFP/
sudo cp /usr/lib/x86_64-linux-gnu/libcuda.so.1 .
echo "#!/usr/bin/env xdg-open
[Desktop Entry]
Name=WJ-ISFP
Type=Application
Icon=/home/$USER/WJ-ISFP/Configs/Icon/View.png
Exec=sh /home/$USER/WJ-ISFP/Configs/AutoStart.sh">/home/$USER/桌面/WJ-ISFP.desktop
echo "echo $1 |sudo -S /home/$USER/WJ-ISFP/main">/home/$USER/WJ-ISFP/Configs/AutoStart.sh
sudo chmod 777 ~/桌面/WJ-ISFP.desktop
sudo cp ~/桌面/WJ-ISFP.desktop /etc/xdg/autostart/WJ-ISFP.desktop 
sudo chmod 777 /etc/xdg/autostart/WJ-ISFP.desktop


