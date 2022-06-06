#include <QApplication>

#include "ui/main_window.h"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QGuiApplication::setApplicationDisplayName("Calibmar");
  std::unique_ptr<calibmar::MainWindow> mainWindow = std::make_unique<calibmar::MainWindow>();
  mainWindow->show();
  return app.exec();
}