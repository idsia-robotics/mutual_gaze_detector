sudo apt-add-repository -y -n 'deb http://archive.ubuntu.com/ubuntu focal main'
sudo apt-add-repository -y 'deb http://archive.ubuntu.com/ubuntu focal universe'
sudo apt-get install -y libsoundio1
sudo apt-add-repository -r -y -n 'deb http://archive.ubuntu.com/ubuntu focal universe'
sudo apt-add-repository -r -y 'deb http://archive.ubuntu.com/ubuntu focal main'

if ! dpkg -s libk4a1.4 > /dev/null; then
	curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb) > /tmp/libk4a1.4_1.4.1_amd64.deb
	echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | sudo debconf-set-selections
	sudo dpkg -i /tmp/libk4a1.4_1.4.1_amd64.deb
fi
if ! dpkg -s libk4a1.4-dev > /dev/null; then
	curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb > /tmp/libk4a1.4-dev_1.4.1_amd64.deb
	sudo dpkg -i /tmp/libk4a1.4-dev_1.4.1_amd64.deb
fi
if ! dpkg -s libk4abt1.1 > /dev/null; then
	curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb > /tmp/libk4abt1.1_1.1.2_amd64.deb
	echo 'libk4abt1.1	libk4abt1.1/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | sudo debconf-set-selections
	sudo dpkg -i /tmp/libk4abt1.1_1.1.2_amd64.deb
fi
if ! dpkg -s libk4abt1.1-dev > /dev/null; then
	curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb > /tmp/libk4abt1.1-dev_1.1.2_amd64.deb
	sudo dpkg -i /tmp/libk4abt1.1-dev_1.1.2_amd64.deb
fi
if ! dpkg -s k4a-tools > /dev/null; then
	curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb > /tmp/k4a-tools_1.4.1_amd64.deb
	sudo dpkg -i /tmp/k4a-tools_1.4.1_amd64.deb
fi

# Packages versions:
# libk4a1.4                                          1.4.1                                   amd64        Dynamic Libraries for Azure Kinect Runtime
# libk4a1.4-dev                                      1.4.1                                   amd64        Headers and cmake files needed for Azure Kinect Development
# libk4abt1.1                                        1.1.2                                   amd64        Dynamic Libraries for Azure Kinect Body Tracking Runtime
# libk4abt1.1-dev                                    1.1.2                                   amd64        Headers and cmake files needed for Azure Kinect Body Tracking Development
