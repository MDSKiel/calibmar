#include <QApplication>
#include <clocale>

#include "ui/main_window.h"

#include <colmap/controllers/option_manager.h>

int main(int argc, char* argv[]) {
  Q_INIT_RESOURCE(resources);
  QApplication app(argc, argv);
  QGuiApplication::setApplicationDisplayName("Calibmar");
  std::setlocale(LC_ALL, "C");
  std::unique_ptr<calibmar::MainWindow> mainWindow = std::make_unique<calibmar::MainWindow>();
  mainWindow->show();
  return app.exec();
}