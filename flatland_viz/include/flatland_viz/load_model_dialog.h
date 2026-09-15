#ifndef FLATLAND_VIZ_LOAD_MODEL_DIALOG_H
#define FLATLAND_VIZ_LOAD_MODEL_DIALOG_H

#include <QCheckBox>
#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QString>
#include <QWidget>

namespace flatland_viz {
class SpawnModelTool;
}

class LoadModelDialog : public QDialog {
  Q_OBJECT

 public:
  LoadModelDialog(QWidget *parent, flatland_viz::SpawnModelTool *tool);

 private:
  QString ChooseFile();
  void AddNumberAndUpdateName();

  static QString path_to_model_file;
  static int count;
  static bool numbering;

  flatland_viz::SpawnModelTool *tool_;
  QLineEdit *n_edit;
  QLabel *p_label;
  QCheckBox *n_checkbox;

 public Q_SLOTS:
  void NumberCheckBoxChanged(bool value);
  void CancelButtonClicked();
  void OkButtonClicked();
  void on_PathButtonClicked();
};

#endif  // FLATLAND_VIZ_LOAD_MODEL_DIALOG_H
