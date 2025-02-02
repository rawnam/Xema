#include "select_calibration_board_gui.h"

SelectCalibrationBoardGui::SelectCalibrationBoardGui(QWidget* parent)
	: QDialog(parent)
{
	ui.setupUi(this);

	board_type_ = 20;
	confirm_flag_ = false;


	connect(ui.radioButton_board_4, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_board_4(bool)));
	connect(ui.radioButton_board_12, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_board_12(bool)));
	connect(ui.radioButton_board_20, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_board_20(bool)));
	connect(ui.radioButton_board_40, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_board_40(bool)));
	connect(ui.radioButton_board_80, SIGNAL(toggled(bool)), this, SLOT(do_QRadioButton_toggled_board_80(bool)));

	connect(ui.pushButton_ok, SIGNAL(clicked()), this, SLOT(do_pushButton_ok()));
	connect(ui.pushButton_cancel, SIGNAL(clicked()), this, SLOT(do_pushButton_cancel()));

	do_QRadioButton_toggled_board_20(true);
	ui.retranslateUi(this);
}

SelectCalibrationBoardGui::~SelectCalibrationBoardGui()
{
}


void SelectCalibrationBoardGui::set_board_type(int board)
{
	board_type_ = board;

	switch (board)
	{
	case 4:
	{
		ui.radioButton_board_4->setChecked(true);
		do_QRadioButton_toggled_board_4(true);
	}
	break;
	case 12:
	{
		ui.radioButton_board_12->setChecked(true);
		do_QRadioButton_toggled_board_12(true);
	}
	break;
	case 20:
	{
		ui.radioButton_board_20->setChecked(true);
		do_QRadioButton_toggled_board_20(true);
	}
	break;

	case 40:
	{
		ui.radioButton_board_40->setChecked(true);
		do_QRadioButton_toggled_board_40(true);
	}
	break;
	case 80:
	{
		ui.radioButton_board_80->setChecked(true);
		do_QRadioButton_toggled_board_80(true);
	}
	break;
	default:
		break;
	}
}

int SelectCalibrationBoardGui::get_board_type()
{
	return board_type_;
}


bool SelectCalibrationBoardGui::is_confirm()
{
	return confirm_flag_;
}

void SelectCalibrationBoardGui::do_QRadioButton_toggled_board_4(bool state)
{
	board_type_ = 4;

	QString str = "";
	str += tr("  Asymmetric Circle Calibration Board");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Number of Circles: 7*11");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Circle Spacing: 4 mm");

	ui.label_board_message->setText(str);
}

void SelectCalibrationBoardGui::do_QRadioButton_toggled_board_12(bool state)
{
	board_type_ = 12;

	QString str = "";
	str += tr("  Asymmetric Circle Calibration Board");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Number of Circles: 7*11");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Circle Spacing: 12 mm");

	ui.label_board_message->setText(str);
}

void SelectCalibrationBoardGui::do_QRadioButton_toggled_board_20(bool state)
{
	board_type_ = 20;

	QString str = "";
	str += tr("  Asymmetric Circle Calibration Board");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Number of Circles: 7*11");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Circle Spacing: 20 mm");

	ui.label_board_message->setText(str);
}



void SelectCalibrationBoardGui::do_QRadioButton_toggled_board_40(bool state)
{
	board_type_ = 40;

	QString str = "";
	str += tr("  Asymmetric Circle Calibration Board");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Number of Circles: 7*11");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Circle Spacing: 40 mm");

	ui.label_board_message->setText(str);
}

void SelectCalibrationBoardGui::do_QRadioButton_toggled_board_80(bool state)
{
	board_type_ = 80;

	QString str = "";
	str += tr("  Asymmetric Circle Calibration Board");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Number of Circles: 9*13");
	str += "\r\n";
	str += "\r\n";
	str += tr("  Circle Spacing: 80 mm");

	ui.label_board_message->setText(str);
}

void SelectCalibrationBoardGui::do_pushButton_ok()
{
	confirm_flag_ = true;

	this->accept();
}

void SelectCalibrationBoardGui::do_pushButton_cancel()
{
	confirm_flag_ = false;
	this->reject();
}