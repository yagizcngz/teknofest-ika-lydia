# Kayar Engel - Gazebo Entegrasyonu (ROS 2 Humble)

## Dosya Yapısı

```
kayar_engel/
├── model.config            # Gazebo model tanımı
├── model.sdf               # Engel modeli (1m genişlik, prismatic joint)
├── kayar_engel_plugin.cc   # C++ hareket plugin'i (20 cm/s git-gel)
├── CMakeLists.txt          # Derleme dosyası
├── ornek_parkur.world      # Örnek world dosyası
└── KURULUM.md              # Bu dosya
```

## 1. Plugin'i Derle

```bash
# Derleme dizini oluştur
cd kayar_engel
mkdir build && cd build

# CMake ile yapılandır ve derle
cmake ..
make
```

## 2. Gazebo'ya Tanıt

```bash
# Plugin kütüphanesini Gazebo'nun bulabilmesi için:
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)/build

# Modeli Gazebo'nun bulabilmesi için:
# kayar_engel klasörünü ~/.gazebo/models/ altına kopyala
cp -r kayar_engel ~/.gazebo/models/
```

## 3. Mevcut Parkuruna Ekleme

Kendi .world dosyanda, kayar engeli eklemek istediğin yere şunu ekle:

```xml
<include>
  <uri>model://kayar_engel</uri>
  <pose>X Y 0 0 0 YAW</pose>
  <name>kayar_engel_6</name>
</include>
```

**pose değerlerini ayarla:**
- `X`: Parkur ekseni boyunca engelin konumu (metre)
- `Y`: Parkurun ortası (genellikle 0)
- `YAW`: Engelin yönü (parkura dik olmalı, radyan cinsinden)

## 4. Çalıştır

```bash
# Direkt world dosyasıyla:
gazebo ornek_parkur.world --verbose

# Veya ROS 2 launch ile:
ros2 launch gazebo_ros gazebo.launch.py world:=/tam/yol/ornek_parkur.world
```

## 5. Parametreleri Değiştirme

model.sdf içindeki plugin parametrelerini değiştirebilirsin:

```xml
<plugin name="kayar_engel_hareketi" filename="libkayar_engel_plugin.so">
  <speed>0.20</speed>              <!-- Hız: 0.20 = 20 cm/s -->
  <lower_limit>-1.5</lower_limit>  <!-- Sol sınır (metre) -->
  <upper_limit>1.5</upper_limit>   <!-- Sağ sınır (metre) -->
</plugin>
```

## 6. Plugin Olmadan Alternatif (Hızlı Test)

Plugin derlemeden hızlı test yapmak istersen, model.sdf'deki
plugin satırını kaldır ve bunun yerine `<static>false</static>`
bırakarak ROS 2 topic üzerinden joint'i kontrol et:

```bash
# Terminal 1: Gazebo'yu başlat
gazebo ornek_parkur.world

# Terminal 2: Joint'e hız komutu gönder
ros2 topic pub /kayar_engel_6/engel_slide/cmd_vel std_msgs/Float64 "data: 0.2"
```

## Şartname Özeti (6.8 Kayar Engel)

| Özellik | Değer |
|---------|-------|
| Engel genişliği | 1 metre |
| Hareket hızı | 20 cm/s |
| Hareket tipi | Sürekli git-gel (sağ-sol) |
| Yön değiştirme | Beklemeden anlık |
| Parkur genişliği | 3 metre |
