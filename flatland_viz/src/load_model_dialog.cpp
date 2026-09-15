#include "flatland_viz/load_model_dialog.h"

#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

#include <boost/filesystem.hpp>

#include "flatland_viz/spawn_model_tool.h"

QString LoadModelDialog::path_to_model_file;
int LoadModelDialog::count;
bool LoadModelDialog::numbering;

LoadModelDialog::LoadModelDialog(QWidget *parent,
                                 flatland_viz::SpawnModelTool *tool)
    : QDialog(parent), tool_(tool) {
  QVBoxLayout *v_layout = new QVBoxLayout;
  setLayout(v_layout);

  QHBoxLayout *h0_layout = new QHBoxLayout;
  QHBoxLayout *h1_layout = new QHBoxLayout;
  QHBoxLayout *h2_layout = new QHBoxLayout;
  QHBoxLayout *h3_layout = new QHBoxLayout;

  QPushButton *pathButton = new QPushButton("choose file");
  p_label = new QLabel;
  n_checkbox = new QCheckBox;
  n_edit = new QLineEdit;
  QPushButton *okButton = new QPushButton("ok");
  QPushButton *cancelButton = new QPushButton("cancel");

  pathButton->setFocusPolicy(Qt::NoFocus);
  p_label->setFocusPolicy(Qt::NoFocus);
  n_checkbox->setFocusPolicy(Qt::NoFocus);
  n_edit->setFocusPolicy(Qt::ClickFocus);
  okButton->setFocusPolicy(Qt::NoFocus);
  cancelButton->setFocusPolicy(Qt::NoFocus);

  connect(pathButton, &QAbstractButton::clicked, this,
          &LoadModelDialog::on_PathButtonClicked);
  connect(okButton, &QAbstractButton::clicked, this,
          &LoadModelDialog::OkButtonClicked);
  connect(cancelButton, &QAbstractButton::clicked, this,
          &LoadModelDialog::CancelButtonClicked);
  connect(n_checkbox, &QAbstractButton::clicked, this,
          &LoadModelDialog::NumberCheckBoxChanged);

  h0_layout->addWidget(pathButton);

  p_label->setText(path_to_model_file);
  h1_layout->addWidget(new QLabel("path:"));
  h1_layout->addWidget(p_label);

  h2_layout->addWidget(new QLabel("number:"));
  h2_layout->addWidget(n_checkbox);
  n_checkbox->setChecked(numbering);
  h2_layout->addWidget(new QLabel("name:"));
  h2_layout->addWidget(n_edit);

  AddNumberAndUpdateName();

  h3_layout->addWidget(okButton);
  h3_layout->addWidget(cancelButton);

  v_layout->addLayout(h0_layout);
  v_layout->addLayout(h1_layout);
  v_layout->addLayout(h2_layout);
  v_layout->addLayout(h3_layout);

  setLayout(v_layout);
  this->setAttribute(Qt::WA_DeleteOnClose, true);
}

void LoadModelDialog::CancelButtonClicked() { this->close(); }

void LoadModelDialog::OkButtonClicked() {
  QString name = n_edit->displayText();
  tool_->SaveName(name);
  tool_->SavePath(path_to_model_file);
  tool_->BeginPlacement();
  this->close();
}

void LoadModelDialog::AddNumberAndUpdateName() {
  std::string bsfn =
      boost::filesystem::path(path_to_model_file.toStdString()).stem().string();
  QString name = QString::fromStdString(bsfn);

  if (numbering) {
    name = name.append(QString::number(count++));
  }

  n_edit->setText(name);
}

void LoadModelDialog::on_PathButtonClicked() {
  path_to_model_file = ChooseFile();
  AddNumberAndUpdateName();
  p_label->setText(path_to_model_file);
  n_edit->setFocus();
}

void LoadModelDialog::NumberCheckBoxChanged(bool i) {
  (void)i;
  numbering = !numbering;
  AddNumberAndUpdateName();
}

QString LoadModelDialog::ChooseFile() {
  QString fileName =
      QFileDialog::getOpenFileName(nullptr, tr("Open model file"), "", "");
  if (fileName.isEmpty()) {
    return fileName;
  }
  QFile file(fileName);
  if (!file.open(QIODevice::ReadOnly)) {
    QMessageBox::information(nullptr, tr("Unable to open file"),
                             file.errorString());
    return fileName;
  }
  file.close();
  return fileName;
}
