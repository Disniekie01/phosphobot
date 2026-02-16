import { Layout } from "@/components/layout/layout";
import { AdminPage } from "@/pages/AdminSettingsPage";
import { BrowsePage } from "@/pages/BrowsePage";
import { DashboardPage } from "@/pages/DashboardPage";
import { NetworkPage } from "@/pages/NetworkPage";
import { ViewVideoPage } from "@/pages/ViewVideoPage";
import { AuthForm } from "@/pages/auth/AuthForm";
import { ConfirmCode } from "@/pages/auth/ConfirmCode";
import { ConfirmEmail } from "@/pages/auth/ConfirmEmail";
import { ForgotPassword } from "@/pages/auth/ForgotPassword";
import { ResetPassword } from "@/pages/auth/ResetPassword";
import { ROS2BridgePage } from "@/pages/ROS2BridgePage";
import { CalibrationPage } from "@/pages/calibration/CalibrationPage";
import { ControlPage } from "@/pages/control/ControlPage";
import { MobileControlPage } from "@/pages/control/MobileControlPage";
import { Route, BrowserRouter as Router, Routes } from "react-router-dom";

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Layout />}>
          <Route index element={<DashboardPage />} />
          <Route path="/control" element={<ControlPage />} />
          <Route path="/mobile" element={<MobileControlPage />} />
          <Route path="/browse" element={<BrowsePage />} />
          <Route path="/browse/:path" element={<BrowsePage />} />
          <Route path="/admin" element={<AdminPage />} />
          <Route path="/calibration" element={<CalibrationPage />} />
          <Route path="/network" element={<NetworkPage />} />
          <Route path="/ros2" element={<ROS2BridgePage />} />
          <Route path="/viz" element={<ViewVideoPage />} />
          <Route path="/auth" element={<AuthForm />} />
          <Route path="/sign-in" element={<AuthForm />} />
          <Route path="/sign-up" element={<AuthForm />} />
          <Route path="/sign-up/confirm" element={<ConfirmCode />} />
          <Route path="/auth/confirm" element={<ConfirmEmail />} />
          <Route path="/auth/forgot-password" element={<ForgotPassword />} />
          <Route path="/auth/reset-password" element={<ResetPassword />} />
        </Route>
      </Routes>
    </Router>
  );
}

export default App;
