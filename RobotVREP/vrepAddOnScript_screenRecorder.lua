--execMode=...
--if(execMode == 0) then 
--simSetBooleanParameter(sim_boolparam_video_recording_triggered,1)
--simSetBooleanParameter(sim_boolparam_browser_visible,false)
--simSetBooleanParameter(sim_boolparam_hierarchy_visible,false)
--end

p = simGetStringSignal('videoPath')

if p then
	simClearStringSignal('videoPath')
	simSetStringParameter(sim_stringparam_video_filename,p)
end