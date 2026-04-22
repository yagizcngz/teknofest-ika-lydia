/**
 * TEKNOFEST 2026 IKA - Kayar Engel Gazebo Plugin
 *
 * Bu plugin, kayar engeli 20 cm/s hizla saga-sola
 * surekli git-gel hareketi yaptirir.
 *
 * Sartname kurallari:
 *   - Hiz: 20 cm/s (sabit)
 *   - Genislik: 1 metre
 *   - Parkur disina ciktiginda beklemeden ters yone doner
 *   - Surekli rejimde calisir
 */

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class KayarEngelPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      this->model = _model;

      // "engel_slide" prismatic joint'i bul
      this->joint = this->model->GetJoint("engel_slide");
      if (!this->joint)
      {
        gzerr << "KayarEngelPlugin: 'engel_slide' joint bulunamadi!\n";
        return;
      }

      // Parametreleri SDF'den oku (opsiyonel override)
      this->speed = 0.20;  // 20 cm/s varsayilan
      if (_sdf->HasElement("speed"))
        this->speed = _sdf->Get<double>("speed");

      this->lower_limit = -1.5;
      this->upper_limit = 1.5;
      if (_sdf->HasElement("lower_limit"))
        this->lower_limit = _sdf->Get<double>("lower_limit");
      if (_sdf->HasElement("upper_limit"))
        this->upper_limit = _sdf->Get<double>("upper_limit");

      // Baslangic yonu: pozitif (saga)
      this->direction = 1.0;

      // Her simulasyon adiminda OnUpdate fonksiyonunu cagir
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&KayarEngelPlugin::OnUpdate, this));

      gzmsg << "KayarEngelPlugin yuklendi. Hiz: " << this->speed
            << " m/s, Sinirlar: [" << this->lower_limit
            << ", " << this->upper_limit << "]\n";
    }

    void OnUpdate()
    {
      // Mevcut joint pozisyonunu oku
      double position = this->joint->Position(0);

      // Sinira ulastiginda yon degistir (beklemeden!)
      if (position >= this->upper_limit)
      {
        this->direction = -1.0;
      }
      else if (position <= this->lower_limit)
      {
        this->direction = 1.0;
      }

      // Joint'e sabit hiz uygula
      this->joint->SetVelocity(0, this->direction * this->speed);
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    event::ConnectionPtr updateConnection;

    double speed;        // m/s cinsinden hiz
    double direction;    // +1.0 veya -1.0
    double lower_limit;  // sol sinir (metre)
    double upper_limit;  // sag sinir (metre)
  };

  // Plugin'i Gazebo'ya kaydet
  GZ_REGISTER_MODEL_PLUGIN(KayarEngelPlugin)
}
