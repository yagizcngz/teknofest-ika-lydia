# Kayar Engel - Parkura Entegrasyon Talimati

## Dosyalar

```
kayar_engel_model/
  model.config          # Gazebo model tanimi
  model.sdf             # Engel modeli (ROS 2 uyumlu, plugin derleme gerektirmez)
kayar_engel_hareket.py  # Python hareket kontrolcu node
kayar_engel_plugin.cc   # (Opsiyonel) C++ plugin - derleme gerektirir
CMakeLists.txt          # (Opsiyonel) C++ plugin icin
```

## Adim 1: Modeli Gazebo'ya Tani

kayar_engel_model klasorunu teknofest_ika paketinin models klasorune kopyala:

```bash
cp -r kayar_engel_model ~/ika_ws/src/teknofest_ika/models/kayar_engel
```

## Adim 2: World Dosyana Ekle

Kendi .world dosyanda `</world>` etiketinden ONCE su blogu ekle:

```xml
<!-- KAYAR ENGEL - Sartname 6.8 -->
<model name="kayar_engel">
  <include>
    <uri>model://kayar_engel</uri>
  </include>
  <!--
    KONUM AYARI:
    Asagidaki pose degerlerini kendi parkuruna gore ayarla.
    Format: x y z roll pitch yaw

    ONEMLI: Parkurun yonune gore yaw degerini ayarla:
      - Parkur X ekseninde ilerliyorsa: yaw = 0
      - Parkur Y ekseninde ilerliyorsa: yaw = 1.5708 (90 derece)

    Engel, Y ekseni boyunca saga-sola hareket eder.
    Eger parkurun o bolumu X ekseni boyunca uzaniyorsa,
    yaw'i 90 derece (1.5708 rad) cevir ki engel
    parkura dik hareket etsin.
  -->
  <pose>X_DEGERI Y_DEGERI 0 0 0 YAW_DEGERI</pose>
</model>
```

## Adim 3: Koordinatlari Bul

Gazebo'da parkurunu ac, kukalardan (trafik konileri) hemen sonraki
duz bolgenin ortasina mouse ile git. Sol alttaki koordinat cubugunda
X ve Y degerlerini oku. Bu degerleri yukaridaki pose'a yaz.

## Adim 4: Hareket Node'unu Calistir

### Yontem A: Dogrudan calistir (en kolay)
```bash
# Terminal 1: Gazebo'yu baslat
ros2 launch teknofest_ika gazebo.launch.py

# Terminal 2: Hareket kontrolcuyu baslat
cd ~/Desktop/kayar_engel
python3 kayar_engel_hareket.py
```

### Yontem B: Launch dosyasina ekle
Kendi launch dosyana su node'u ekle:
```python
from launch_ros.actions import Node as LaunchNode

kayar_engel_node = LaunchNode(
    package='teknofest_ika',  # veya hangi paketse
    executable='kayar_engel_hareket.py',
    name='kayar_engel_kontrol',
    parameters=[{
        'hiz': 0.20,
        'sol_sinir': -1.5,
        'sag_sinir': 1.5,
    }]
)
```

## Adim 5: Test Et

```bash
# Joint durumunu kontrol et
ros2 topic echo /kayar_engel/joint_states

# Manuel hiz komutu gonder (test icin)
ros2 topic pub /kayar_engel/engel_slide/cmd_vel std_msgs/msg/Float64 "data: 0.2"
```

## Sorun Giderme

**Engel hareket etmiyor:**
- `ros2 topic list` ile topic'lerin gorunup gorunmedigini kontrol et
- Model `static: false` olmali (model.sdf'de kontrol et)
- Gazebo'nun fizik motorunun calistigini kontrol et (pause'da olmamali)

**Engel cok hizli/yavas:**
- `kayar_engel_hareket.py` icindeki `hiz` parametresini degistir
- Varsayilan: 0.20 m/s (20 cm/s - sartname degeri)

**Engel yanlis yonde hareket ediyor:**
- World dosyasindaki `yaw` degerini 1.5708 (90 derece) cevir
- Veya model.sdf'deki `<xyz>0 1 0</xyz>` satirini `<xyz>1 0 0</xyz>` olarak degistir
