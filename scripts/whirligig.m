function [p] = whirligig(PDD, LSD)

% %     p1 | p2 | signals/moving
% %     --------------------
% %     0  | 0  | stop
% %     0  | 1  | left
% %     1  | 0  | right 
% %     1  | 1  | forward

if PDD(1) == 0 && PDD(2) == 0 && PDD(8) == 0
    p = [1 1];
elseif (PDD(7) == 1 || PDD(6) == 1) && sum(PDD) < 4
    p = [0 1];
else
    p = [1 0];
end