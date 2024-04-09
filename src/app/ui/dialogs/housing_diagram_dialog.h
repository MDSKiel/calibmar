
#pragma once

#include <QtCore>
#include <QtWidgets>

#include "ui/widgets/housing_diagram_widget.h"

namespace calibmar {

  class HousingDiagramDialog : public QDialog {
   public:
    HousingDiagramDialog(QWidget* parent = nullptr);

   private:
    HousingDiagramWidget* housing_diagram_;
  };
}
