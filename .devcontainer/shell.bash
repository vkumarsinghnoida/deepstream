apt update
apt install sudo -y
sudo apt install openssh-server python3 python3-pip git curl wget nano -y && sudo service ssh start
pip install -r requirements.txt
