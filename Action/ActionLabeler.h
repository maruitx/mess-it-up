#ifndef ACTIONLABELER_H
#define ACTIONLABELER_H

#include "../Utilities/utility.h"

class ActionLabelerWidget;
class DepthSensor;
class CScene;

class ActionLabeler : public QObject
{
	Q_OBJECT

public:
	ActionLabeler(DepthSensor *depthSensor, CScene *scene, QObject *parent = 0);
	~ActionLabeler();
	 
	QVector<int> getSelectedModelIDList(const QVector<QString> &nameList);

	void updateSelectModelInWidget(QVector<int> selectIDs);

	bool hasWidget() { return m_hasWidget; };
	
public:
	QVector<QString> allModelNameList;
	QVector<int> selectModelIDs;

public slots:
	void createWidget();
	void updateFrameInfo();
	void updateSelectModelInScene();

	void clearSelectedLabels();

	void addLabelToCurrFrame();
	void clearLabelOfCurrFrame();

	void addLabelToFrames();
	void clearLabelOfFrames();

	void changeToFrame(int i);

	void saveLabelFile();
	void clearAllLabels();

private:
	ActionLabelerWidget *m_widget;
	DepthSensor *m_depthSensor;
	CScene *m_scene;

	bool m_hasWidget;
	
};

#endif // ACTIONLABELER_H
