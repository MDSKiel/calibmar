#include "housing_diagram_dialog.h"
#include "ui/widgets/zoomable_scroll_area.h"

namespace {
  template <typename Func>
  void AddDoubleSlider(QGridLayout* parent, double min, double max, double value, const std::string& label_string, Func func) {
    QSlider* slider = new QSlider(Qt::Orientation::Horizontal);
    slider->setMaximum(static_cast<int>(max * 1000));
    slider->setMinimum(static_cast<int>(min * 1000));
    slider->setValue(static_cast<int>(value * 1000));
    QLabel* value_label = new QLabel(QString::number(value));
    value_label->setMinimumWidth(value_label->fontMetrics().horizontalAdvance("000.000"));
    QLabel* label = new QLabel(QString::fromStdString(label_string));

    slider->connect(slider, &QSlider::valueChanged, slider, [value_label, parent, func = std::move(func)](int v) {
      double value = v / 1000.0;
      value_label->setText(QString::number(value));
      func(value);
    });
    int row = parent->rowCount();
    parent->addWidget(label, row, 0);
    parent->addWidget(slider, row, 1);
    parent->addWidget(value_label, row, 2);
  }

  template <typename Func>
  void AddIntSlider(QGridLayout* parent, int min, int max, int value, const std::string& label_string, Func func) {
    QSlider* slider = new QSlider(Qt::Orientation::Horizontal);
    slider->setMaximum(max);
    slider->setMinimum(min);
    slider->setValue(value);
    QLabel* value_label = new QLabel(QString::number(value));
    value_label->setMinimumWidth(value_label->fontMetrics().horizontalAdvance("000.000"));
    QLabel* label = new QLabel(QString::fromStdString(label_string));

    slider->connect(slider, &QSlider::valueChanged, slider, [value_label, parent, func = std::move(func)](int v) {
      value_label->setText(QString::number(v));
      func(v);
    });
    int row = parent->rowCount();
    parent->addWidget(label, row, 0);
    parent->addWidget(slider, row, 1);
    parent->addWidget(value_label, row, 2);
  }
}

namespace calibmar {
  HousingDiagramDialog::HousingDiagramDialog(QWidget* parent) {
    setWindowTitle("Housing Diagram");

    QVBoxLayout* layout = new QVBoxLayout(this);
    QSplitter* splitter = new QSplitter(this);

    // image view
    ZoomableScrollArea* area = new ZoomableScrollArea();
    housing_diagram_ = new HousingDiagramWidget(area, {1000, 1000});
    housing_diagram_->setMinimumSize(500, 500);
    area->setMinimumSize(550, 550);
    area->setWidget(housing_diagram_);
    housing_diagram_->resize(1000, 1000);
    // side
    QWidget* side = new QWidget();
    QVBoxLayout* side_parent_layout = new QVBoxLayout(side);
    QGridLayout* sliders_side_layout = new QGridLayout();

    QComboBox* diagram_type_box = new QComboBox(this);
    diagram_type_box->addItem("Flat Port");
    diagram_type_box->addItem("Dome Port");
    connect(diagram_type_box, QOverload<int>::of(&QComboBox::currentIndexChanged), diagram_type_box,
            [diagram = housing_diagram_](int index) {
      if (index == 0) {
        diagram->SetDiagramType(HousingDiagramWidget::DiagramType::FlatPort);
        diagram->update();
      }
      else {
        diagram->SetDiagramType(HousingDiagramWidget::DiagramType::DomePort);
        diagram->update();
      }
    });

    // FOV
    AddDoubleSlider(sliders_side_layout, 10, 170, 75, "FOV", [diagram = housing_diagram_](double v) {
      diagram->SetFOVDegrees(v);
      diagram->update();
    });
    // Ray Count
    AddIntSlider(sliders_side_layout, 3, 20, 7, "Ray Count", [diagram = housing_diagram_](double v) {
      diagram->SetRayCount(v);
      diagram->update();
    });
    // na ng nw
    AddDoubleSlider(sliders_side_layout, 1, 3, 1, "n air", [diagram = housing_diagram_](double v) {
      diagram->SetRefrIndexAir(v);
      diagram->update();
    });
    AddDoubleSlider(sliders_side_layout, 1, 3, 1.47, "n glass", [diagram = housing_diagram_](double v) {
      diagram->SetRefrIndexGlass(v);
      diagram->update();
    });
    AddDoubleSlider(sliders_side_layout, 1, 3, 1.334, "n water", [diagram = housing_diagram_](double v) {
      diagram->SetRefrIndexWater(v);
      diagram->update();
    });

    side_parent_layout->addWidget(diagram_type_box);
    side_parent_layout->addLayout(sliders_side_layout);
    side_parent_layout->addStretch();

    splitter->addWidget(area);
    splitter->addWidget(side);
    layout->addWidget(splitter);
  }
}