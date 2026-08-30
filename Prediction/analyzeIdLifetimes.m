S = load('m1_perception_day1.mat');
frames = S.perceptionData.signals.values;



idFirstSeen = containers.Map('KeyType','double','ValueType','double');
idLastSeen  = containers.Map('KeyType','double','ValueType','double');

for f = 1:numel(frames)
    tracks = frames(f).tracks;
    n = frames(f).num_tracks;
    for i = 1:n
        if ~tracks(i).valid
            continue
        end
        tid = double(tracks(i).id);
        if ~isKey(idFirstSeen, tid)
            idFirstSeen(tid) = f;
        end
        idLastSeen(tid) = f;
    end
end

idFrames = containers.Map('KeyType','double','ValueType','any');

for f = 1:numel(frames)
    tracks = frames(f).tracks;
    n = frames(f).num_tracks;
    for i = 1:n
        if ~tracks(i).valid
            continue
        end
        tid = double(tracks(i).id);
        if ~isKey(idFrames, tid)
            idFrames(tid) = f;
        else
            idFrames(tid) = [idFrames(tid), f];
        end
    end
end

ids2 = cell2mat(keys(idFrames));
gapCount = 0;
for k = 1:numel(ids2)
    seq = idFrames(ids2(k));
    if any(diff(seq) > 1)
        gapCount = gapCount + 1;
        fprintf('id %d has a GAP — frames: %s\n', ids2(k), mat2str(seq));
    end
end
fprintf('Total ids with gaps: %d out of %d\n', gapCount, numel(ids2));
%% % Pick a short-lived id to test — id 42 disappears after frame 17
testId = 42;
testFrame = idLastSeen(testId);  % last frame it appeared
nextFrame = testFrame + 1;

tracks_next = frames(nextFrame).tracks;
n_next = frames(nextFrame).num_tracks;

dt = 1;
preds_next = predictMotionCVCTRBlock(tracks_next, n_next, dt);

% Confirm id 42 is truly gone from the input at nextFrame
stillPresent = false;
for i = 1:n_next
    if double(tracks_next(i).id) == testId
        stillPresent = true;
    end
end
fprintf('id %d still present at frame %d? %d\n', testId, nextFrame, stillPresent);

% Check all slots beyond n_next are correctly zeroed
for i = (n_next+1):numel(preds_next)
    if preds_next(i).valid
        fprintf('WARNING: slot %d marked valid but should be empty (num_tracks=%d)\n', i, n_next);
    end
end
disp('Slot-clearing check complete.');