sudo iptables -A INPUT -i wlxd03745f79670 -p udp --dport 11111 -j ACCEPT
sudo iptables -A INPUT -i wlxd03745f79670 -p udp --dport 11111 -j ACCEPT
sudo iptables -A PREROUTING -t nat -i wlxd03745f79670 -p udp --dport 11111 -j REDIRECT --to-port 11111

sudo iptables -A INPUT -i wlxd0374572e205 -p udp --dport 11111 -j ACCEPT
sudo iptables -A INPUT -i wlxd0374572e205 -p udp --dport 11113 -j ACCEPT
sudo iptables -A PREROUTING -t nat -i wlxd0374572e205 -p udp --dport 11111 -j REDIRECT --to-port 11113

sudo iptables -A INPUT -i wlx6c5ab04a495e -p udp --dport 11111 -j ACCEPT
sudo iptables -A INPUT -i wlx6c5ab04a495e -p udp --dport 11115 -j ACCEPT
sudo iptables -A PREROUTING -t nat -i wlx6c5ab04a495e -p udp --dport 11111 -j REDIRECT --to-port 11115
