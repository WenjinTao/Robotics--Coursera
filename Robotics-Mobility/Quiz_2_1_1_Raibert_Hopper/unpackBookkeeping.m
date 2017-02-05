% ==============================================================
% utility function for packing/unpacking/passing data
% ==============================================================
function [touchdownTime, liftoffTime] = unpackBookkeeping(bookkeeping)
    touchdownTime = bookkeeping.touchdownTime;
    liftoffTime = bookkeeping.liftoffTime;
end