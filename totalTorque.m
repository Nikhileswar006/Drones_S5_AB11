function tau = totalTorque(tau1, tau2)
% TOTALTORQUE Computes total control torque tau = tau1 + tau2 (Eq. 23).
    tau = tau1(:) + tau2(:);
end
