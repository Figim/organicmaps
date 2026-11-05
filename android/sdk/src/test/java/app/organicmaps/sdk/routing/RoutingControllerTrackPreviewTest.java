package app.organicmaps.sdk.routing;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.mockStatic;

import app.organicmaps.sdk.Router;
import app.organicmaps.sdk.util.log.Logger;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.List;
import org.junit.Test;
import org.mockito.MockedStatic;

public class RoutingControllerTrackPreviewTest
{
  @Test
  public void newTrackBuildDropsDeferredCancelledResult() throws ReflectiveOperationException
  {
    final RoutingController controller = new RoutingController() {
      @Override
      public boolean isFollowingTrack()
      {
        return false;
      }
    };
    final RoutingController.Container phone = mock(RoutingController.Container.class);
    set(controller, "mLastResultCode", ResultCodes.CANCELLED);
    set(controller, "mContainsCachedResult", true);
    set(controller, "mLastRouterType", Router.Pedestrian);
    setState(controller, "PREPARE");
    controller.attach(phone);
    controller.onSaveState(phone);
    controller.detach(phone);
    // The JNI acceptance is covered by native tests; exercise its Java transaction boundary
    // without attempting to mock uninstrumentable native methods in a host JVM.
    Method begin = RoutingController.class.getDeclaredMethod("onCoreStartedTrackBuild");
    begin.setAccessible(true);
    try (MockedStatic<Logger> ignored = mockStatic(Logger.class))
    {
      begin.invoke(controller);
      controller.attach(phone);
      controller.restore();
    }
    assertTrue(controller.isPlanning());
    assertTrue(controller.isBuilding());
  }

  @Test
  public void roadOptionsDoNotRebuildGeometricTrackPreview() throws ReflectiveOperationException
  {
    class TrackController extends RoutingController
    {
      boolean rebuilt;
      @Override
      public boolean isFollowingTrack()
      {
        return true;
      }
      @Override
      public void rebuildLastRoute()
      {
        rebuilt = true;
      }
    }
    TrackController controller = new TrackController();
    setState(controller, "PREPARE");
    controller.attach(mock(RoutingController.Container.class));
    controller.onRoutingOptionsChanged();
    assertFalse(controller.rebuilt);
    assertTrue(controller.isPlanning());
  }

  @Test
  public void staleRoadRebuildKeepsTheBuiltTrackPreview() throws ReflectiveOperationException
  {
    RoutingController controller = new RoutingController() {
      @Override
      public boolean isFollowingTrack()
      {
        return true;
      }
    };
    setState(controller, "PREPARE");
    set(controller, "mBuildState", RoutingController.BuildState.BUILT);
    controller.rebuildLastRoute();
    assertTrue(controller.isPlanning());
    assertEquals(RoutingController.BuildState.BUILT, controller.getBuildState());
  }

  @Test
  public void acceptedChartPickBecomesTheEntry() throws ReflectiveOperationException
  {
    final EntryController controller = new EntryController(FollowTrackResult.Ok);
    setState(controller, "PREPARE");
    set(controller, "mBuildState", RoutingController.BuildState.BUILT);
    set(controller, "mFollowedTrackEntryDistance", 100.0);
    try (MockedStatic<Logger> ignored = mockStatic(Logger.class))
    {
      assertEquals(FollowTrackResult.Ok, controller.updateFollowedTrackEntryAt(250.0));
    }
    assertEquals(250.0, controller.getFollowedTrackEntryDistance(), 0.0);
    assertTrue(controller.isPlanning());
    assertTrue(controller.isBuilding());
  }

  @Test
  public void refusedChartPickKeepsTheEntry() throws ReflectiveOperationException
  {
    try (MockedStatic<Logger> ignored = mockStatic(Logger.class))
    {
      for (FollowTrackResult result : FollowTrackResult.values())
      {
        if (result == FollowTrackResult.Ok)
          continue;
        final EntryController controller = new EntryController(result);
        setState(controller, "PREPARE");
        set(controller, "mBuildState", RoutingController.BuildState.BUILT);
        set(controller, "mFollowedTrackEntryDistance", 100.0);
        assertEquals(result, controller.updateFollowedTrackEntryAt(250.0));
        assertEquals(result.toString(), 100.0, controller.getFollowedTrackEntryDistance(), 0.0);
        assertEquals(RoutingController.BuildState.BUILT, controller.getBuildState());
      }
    }
  }

  @Test
  public void entryIsUnknownOutsideATrackFollow() throws ReflectiveOperationException
  {
    final EntryController controller = new EntryController(FollowTrackResult.Ok);
    set(controller, "mFollowedTrackEntryDistance", 100.0);
    assertEquals(100.0, controller.getFollowedTrackEntryDistance(), 0.0);
    // The core can end a follow without a cancel on the Java side, e.g. on a router type change.
    controller.following = false;
    assertTrue(Double.isNaN(controller.getFollowedTrackEntryDistance()));
  }

  @Test
  public void deletedTracksCancelOnlyTheCurrentPreview() throws ReflectiveOperationException
  {
    class Controller extends RoutingController
    {
      FollowedTrackInfo track;
      boolean exists;
      boolean cancelled;
      @Override
      public FollowedTrackInfo getFollowedTrackInfo()
      {
        return track;
      }
      @Override
      boolean hasTrack(long id)
      {
        return exists;
      }
      @Override
      public boolean cancel()
      {
        cancelled = true;
        return true;
      }
    }
    for (String state : List.of("PREPARE", "NAVIGATION", "NONE"))
    {
      for (boolean exists : new boolean[] {true, false})
      {
        Controller controller = new Controller();
        final Constructor<FollowedTrackInfo> constructor =
            FollowedTrackInfo.class.getDeclaredConstructor(long.class, String.class);
        constructor.setAccessible(true);
        controller.track = constructor.newInstance(1L, "Track");
        controller.exists = exists;
        setState(controller, "PREPARE");
        // The deferred callback must inspect the state when it runs, including a journey
        // that started after bookmark publication. Exercise it without the Android looper.
        setState(controller, state);
        final Method changed = RoutingController.class.getDeclaredMethod("cancelDeletedTrackPreview");
        changed.setAccessible(true);
        changed.invoke(controller);
        assertEquals(state.equals("PREPARE") && !exists, controller.cancelled);
      }
    }
  }

  private static final class EntryController extends RoutingController
  {
    final FollowTrackResult result;
    boolean following = true;

    EntryController(FollowTrackResult result)
    {
      this.result = result;
    }

    @Override
    public boolean isFollowingTrack()
    {
      return following;
    }

    @Override
    FollowTrackResult updateCoreFollowedTrackEntryAt(double distanceMeters)
    {
      return result;
    }
  }

  private static void setState(RoutingController controller, String name) throws ReflectiveOperationException
  {
    Field field = RoutingController.class.getDeclaredField("mState");
    field.setAccessible(true);
    for (Object value : field.getType().getEnumConstants())
      if (((Enum<?>) value).name().equals(name))
      {
        field.set(controller, value);
        return;
      }
    throw new IllegalArgumentException(name);
  }

  private static void set(RoutingController controller, String name, Object value) throws ReflectiveOperationException
  {
    Field field = RoutingController.class.getDeclaredField(name);
    field.setAccessible(true);
    field.set(controller, value);
  }
}
