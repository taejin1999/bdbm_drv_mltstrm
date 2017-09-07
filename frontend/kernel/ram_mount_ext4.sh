sudo sh -c "echo "" > /var/log/kern.log"
sudo mkdir -p /usr/share/bdbm_drv
sudo touch /usr/share/bdbm_drv/ftl.dat
sudo touch /usr/share/bdbm_drv/dm.dat

sudo insmod risa_dev_ramdrive.ko
sudo insmod robusta_drv.ko
sudo mkfs -t ext4 -b 4096 /dev/robusta

#sudo tune2fs -o journal_data_writeback /dev/robusta
#sudo tune2fs -O ^has_journal /dev/robusta
#sudo e2fsck -f /dev/robusta

sudo mount -t ext4 -o discard /dev/robusta /media/robusta
