function B = bezierCurve(P0, P1, P2, t)
        B = (1-t).^2 .* P0 + 2.*(1-t).*t.*P1 + t.^2.*P2;
    end

