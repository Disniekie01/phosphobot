import { AppSidebar } from "@/components/common/app-sidebar";
import { SidebarProvider } from "@/components/ui/sidebar";

export function Sidebar() {
  return (
    <div className="hidden md:block w-[250px] glass-subtle bg-sidebar border-0 border-r border-border">
      <SidebarProvider defaultOpen={true}>
        <AppSidebar />
      </SidebarProvider>
    </div>
  );
}
