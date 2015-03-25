#pragma once

#include "StarlabDrawArea.h"
#include "SurfaceMeshPlugins.h"
#include "SurfaceMeshHelper.h"

#include "mess_widget.h"

#include <pcl/visualization/cloud_viewer.h>

#include "Action/ActionLabeler.h"
#include "Action/ActionViewer.h"

class RgbdViewer;
class ActionLearner;
class ActionPredictor;
class CScene;

class mess_mode : public SurfaceMeshModePlugin
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "mess_mode_plugin")
	Q_INTERFACES(ModePlugin)

public:
	mess_mode();
	~mess_mode();

	QIcon icon(){ return QIcon(":/icon/mess_mode.png"); }

	void create();
	void destory();
	void decorate();

	CScene *getScene() { return m_scene; };
	void setSceneBounds();

	// interaction
	bool mousePressEvent(QMouseEvent *event);

	RgbdViewer *rgbdViewer;

	ActionLabeler *actionLabeler;
	ActionLearner *actionLearner;

	ActionPredictor *actionPreditor;
	ActionViewer *actionViewer;

	QPoint mouseClickPos;

	void test();

public slots:
	void loadScene();
	void setSceneShowModel(int state);
	void setSceneShowOBB(int state);
	void setSceneShowRG(int state);
	void setSceneShowModelName(int state);

	void loadActionJob();
	void loadTestScene();
	void openActionLabeler();
	void openActionViewer();

	void syncWithScan(bool state);

private:
	mess_widget * m_widget;
	CScene *m_scene;

	bool m_actionViewerWidgetCreated;
};

