"""Generate a manual-goal view without changing the 5-1 RViz profile."""
import copy


def configure_manual_view(profile):
    profile=copy.deepcopy(profile)
    manager=profile['Visualization Manager']
    # Remove direct Nav2 action tools so all clicks pass the supervised client.
    tools=[tool for tool in manager.get('Tools',[]) if tool.get('Class') not in
           ('rviz_default_plugins/SetGoal','nav2_rviz_plugins/GoalTool',
            'rviz_default_plugins/SetInitialPose','rviz_default_plugins/PublishPoint')]
    def topic(value):
        return {'Value':value,'Depth':5,'Reliability Policy':'Reliable',
                'Durability Policy':'Volatile','History Policy':'Keep Last'}
    tools.extend([{'Class':'rviz_default_plugins/SetGoal','Topic':topic('/lab5/manual_goal')},
                  {'Class':'rviz_default_plugins/PublishPoint','Topic':topic('/clicked_point'),'Single click':True}])
    manager['Tools']=tools
    manager.setdefault('Displays',[]).append({
        'Class':'rviz_default_plugins/Pose','Name':'Selected Manual Goal','Enabled':True,
        'Topic':{'Value':'/lab5/selected_goal','Reliability Policy':'Reliable',
                 'Durability Policy':'Transient Local','History Policy':'Keep Last','Depth':1},
        'Shape':'Arrow','Color':'255; 170; 0','Alpha':1.0})
    manager['Global Options']['Fixed Frame']='map'
    return profile
