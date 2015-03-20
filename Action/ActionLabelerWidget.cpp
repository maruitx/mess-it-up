#include "ActionLabelerWidget.h"
#include "ActionLabeler.h"

ActionLabelerWidget::ActionLabelerWidget(ActionLabeler *labeler, QWidget *parent /*= 0*/):
m_labeler(labeler), ui(new Ui::ActionLabelerWidget)
{
	ui->setupUi(this);

	for (int i = 0; i < ACTION_NUM; i++)
	{
		ui->actionListWidget->addItem(QString(Action_Labels[i]));
	}

	for (int i = 0; i < m_labeler->allModelNameList.size(); i++)
	{
		ui->modelNameListWidget->addItem(m_labeler->allModelNameList[i]);
	}

	connect(ui->modelNameListWidget, SIGNAL(itemSelectionChanged()), m_labeler, SLOT(updateSelectModelInScene()));
	connect(ui->clearSelectionButton, SIGNAL(clicked()), m_labeler, SLOT(clearSelectedLabels()));

	connect(ui->frameBrowseSlider, SIGNAL(valueChanged(int)), m_labeler, SLOT(changeToFrame(int)));

	connect(ui->addSelectToCurrButton, SIGNAL(clicked()), m_labeler, SLOT(addLabelToCurrFrame()));
	connect(ui->clearCurrLabelButton, SIGNAL(clicked()), m_labeler, SLOT(clearLabelOfCurrFrame()));

	connect(ui->addSelectToFramesButton, SIGNAL(clicked()), m_labeler, SLOT(addLabelToFrames()));
	connect(ui->clearLabeOfFramesButton, SIGNAL(clicked()), m_labeler, SLOT(clearLabelOfFrames()));

	connect(ui->saveLabelButton, SIGNAL(clicked()), m_labeler, SLOT(saveLabelFile()));
	connect(ui->clearAllLabelButton, SIGNAL(clicked()), m_labeler, SLOT(clearAllLabels()));
}

ActionLabelerWidget::~ActionLabelerWidget()
{
	delete ui;
}

void ActionLabelerWidget::setCurrFrameID(int i)
{
	ui->frameIdLabel->setText(QString("ID: %1").arg(i));
}

void ActionLabelerWidget::setCurrFrameLabel(QSet<int> labels)
{
	QString labelStr("Label: ");
	
	if (labels.size() == 0)
	{
		labelStr += QString("null");
	}

	else
	{
		QSet<int>::iterator it = labels.begin();

		for (int i = 0; i < labels.size() - 1;i++)
		{
			labelStr += QString(Action_Labels[*it]) + ", ";
			it++;
		}

		labelStr += QString(Action_Labels[*it]);
	}

	ui->frameLabelsLabel->setText(labelStr);
}

void ActionLabelerWidget::setCurrFrameLabel(QSet<FrameLabel> labels)
{
	ui->frameLabelsLabel->setText(QString("Label:"));

	if (labels.size() == 0)
	{
		ui->frameLabelWidget->clear();
		ui->frameLabelWidget->addItem(QString("null"));
	}

	else
	{
		ui->frameLabelWidget->clear();

		QSet<FrameLabel>::iterator it = labels.begin();

		for (int i = 0; i < labels.size(); i++)
		{
			QString labelStr;
			labelStr = QString(Action_Labels[it->first]) + "," + QString(m_labeler->allModelNameList[it->second]);
			ui->frameLabelWidget->addItem(labelStr);

			it++;
		}
	}	
}

QVector<QString> ActionLabelerWidget::getSelectModelNames()
{
	QList<QListWidgetItem*> items = ui->modelNameListWidget->selectedItems();
	
	selectModelNames.resize(items.size());
	
	int i = 0;
	foreach(QListWidgetItem *it, items)
	{
		selectModelNames[i++] = it->text();
	}

	return selectModelNames;
}
