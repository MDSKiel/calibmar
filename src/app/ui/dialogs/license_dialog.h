#pragma once

#include <QtCore>
#include <QtWidgets>

namespace calibmar {

  class LicenseDialog : public QDialog {
   public:
    LicenseDialog(QWidget* parent = nullptr);

   private:
    QString CalibmarLicense();

    QString CeresLicense();
    QString OpenCVLicense();
    QString QtLicense();

    QString COLMAPLicense();
    // COLMAP transitive Licenses
    QString FLANNLicense();
    QString GraclusLicense();
    QString LSDLicense();
    QString PBALicense();
    QString PoissonReconLicense();
    QString SiftGPULicense();
    QString SQLiteLicense();
    QString VLFeatLicense();
  };
}