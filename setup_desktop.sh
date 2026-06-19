#!/bin/bash
# setup_desktop.sh — Masaüstü ikonu ve kısayol kurulumu
# Hem native Ubuntu hem WSL+Windows için çalışır.

set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "[1/3] İkon oluşturuluyor..."
python3 - << PYEOF
from PIL import Image, ImageDraw
img = Image.new('RGBA', (256, 256), (0, 0, 0, 0))
d = ImageDraw.Draw(img)
def rr(draw, xy, r, fill):
    x0,y0,x1,y1=xy
    draw.rectangle([x0+r,y0,x1-r,y1],fill=fill)
    draw.rectangle([x0,y0+r,x1,y1-r],fill=fill)
    for cx,cy in [(x0,y0),(x1-2*r,y0),(x0,y1-2*r),(x1-2*r,y1-2*r)]:
        draw.ellipse([cx,cy,cx+2*r,cy+2*r],fill=fill)
rr(d,[0,0,255,255],40,(30,30,46,255))
d.rounded_rectangle([108,20,148,100],radius=20,fill=(137,180,250,255))
d.rounded_rectangle([70,90,186,122],radius=20,fill=(137,180,250,255))
d.rounded_rectangle([46,106,84,175],radius=18,fill=(116,199,236,255))
d.rounded_rectangle([172,106,210,175],radius=18,fill=(116,199,236,255))
d.ellipse([88,165,168,245],fill=(166,227,161,255))
d.ellipse([108,185,148,225],fill=(30,30,46,255))
d.ellipse([120,197,136,213],fill=(166,227,161,255))
d.ellipse([96,97,120,121],fill=(243,139,168,255))
d.ellipse([136,97,160,121],fill=(243,139,168,255))
img.save('$WS/icon.png')
print("  icon.png oluşturuldu")
PYEOF

echo "[2/3] .desktop dosyası oluşturuluyor..."
mkdir -p ~/.local/share/applications ~/.local/share/icons
cp "$WS/icon.png" ~/.local/share/icons/end-effector.png

DESKTOP_CONTENT="[Desktop Entry]
Version=1.0
Type=Application
Name=End Effector H2515
Comment=Doosan H2515 B-Pillar Zimparalama Sistemi
Exec=python3 $WS/start_robot.py
Icon=$HOME/.local/share/icons/end-effector.png
Terminal=false
Categories=Science;Robotics;
StartupNotify=true"

echo "$DESKTOP_CONTENT" > ~/.local/share/applications/end-effector.desktop
mkdir -p ~/Desktop
echo "$DESKTOP_CONTENT" > ~/Desktop/end-effector.desktop
chmod +x ~/Desktop/end-effector.desktop
update-desktop-database ~/.local/share/applications/ 2>/dev/null || true

echo "[3/3] Windows kısayolu kontrol ediliyor..."
# WSL ortamında Windows masaüstüne de ekle
if command -v powershell.exe &>/dev/null; then
    WIN_ICON="$HOME/AppData/Local/end_effector_icon.ico"
    python3 - << PYEOF2
from PIL import Image
img = Image.open('$WS/icon.png')
sizes = [(16,16),(32,32),(48,48),(256,256)]
img.save('${WS}/icon.ico', format='ICO', sizes=sizes)
print("  icon.ico oluşturuldu")
PYEOF2

    cp "${WS}/icon.ico" "$(wslpath "$(powershell.exe -Command 'echo $env:LOCALAPPDATA' | tr -d '\r')")/end_effector_icon.ico" 2>/dev/null || true

    cat > /tmp/shortcut.ps1 << 'PS'
$WS = New-Object -ComObject WScript.Shell
$SC = $WS.CreateShortcut("$env:USERPROFILE\Desktop\End Effector H2515.lnk")
$SC.TargetPath = "wsl.exe"
$SC.Arguments = "-d Ubuntu-22.04 bash WSPATH/launch.sh"
$SC.IconLocation = "$env:LOCALAPPDATA\end_effector_icon.ico,0"
$SC.WorkingDirectory = "\\wsl.localhost\Ubuntu-22.04\home\$env:USERNAME"
$SC.Description = "Doosan H2515 B-Pillar Zimparalama"
$SC.Save()
PS
    # launch.sh yolunu yerleştir
    sed -i "s|WSPATH|$WS|g" /tmp/shortcut.ps1
    powershell.exe -ExecutionPolicy Bypass -File "$(wslpath -w /tmp/shortcut.ps1)" 2>/dev/null && \
        echo "  Windows masaüstü kısayolu oluşturuldu" || \
        echo "  Windows kısayolu atlandı (WSL değil?)"
else
    echo "  Native Ubuntu — Windows kısayolu atlandı"
fi

echo ""
echo "✓ Kurulum tamamlandı!"
echo "  Ubuntu masaüstü : ~/Desktop/end-effector.desktop"
echo "  Uygulama menüsü : ~/.local/share/applications/end-effector.desktop"
