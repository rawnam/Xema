#pragma once

#include <QDialog>
#include "about.h"
#include "../firmware/version.h"

class AboutGui : public QDialog
{
	Q_OBJECT

public:
	AboutGui(QWidget* parent = Q_NULLPTR);
	~AboutGui();

	void setFirmwareVersion(QString version);

	void AboutGui::updateVersion();

	void AboutGui::updateProductInfo();

private slots:
	void print_log(QString str);

private:
	Ui::AboutGui ui;

	QString firmware_version;
};

